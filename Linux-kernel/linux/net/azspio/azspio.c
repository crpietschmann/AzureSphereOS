/*
 * Copyright (c) Microsoft
 * Based on code:
 * Copyright (c) 2015, Sony Mobile Communications Inc.
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <azure-sphere/security.h>
#include <linux/azspio.h>
#include <linux/module.h>
#include <linux/netlink.h>
#include <linux/termios.h> /* For TIOCINQ/OUTQ */

#include <net/sock.h>

#include "azspio.h"

#define MTU 1024

/**
 * We track "nodes" and "sockets".
 * A "node" is a hardware endpoint, usually the ringbuffer of an
 * IO core.
 * A "socket" is a socket endpoint.
 */

/**
 * struct azspio_hdr -
 * @component_id - the component ID
 * @reserved - a reserved header field that must be 0 for now.
 */
struct azspio_hdr {
	u8 component_id[AZSPIO_COMPONENT_ID_LENGTH];
	u32 reserved;
} __packed;

#define AZSPIO_HDR_SIZE sizeof(struct azspio_hdr)

struct azspio_sock {
	/* WARNING: sk must be the first member */
	struct sock sk;
	struct sockaddr_azspio us;
	struct sockaddr_azspio peer;
	struct list_head item;

	/* RX backlog are SKBs still owned by the sender */
	struct sk_buff_head rx_backlog;
	struct mutex rx_backlog_lock;

	bool throttled;
};

static inline struct azspio_sock *azspio_sk(struct sock *sk)
{
	BUILD_BUG_ON(offsetof(struct azspio_sock, sk) != 0);
	return container_of(sk, struct azspio_sock, sk);
}

/* An all-zero component ID */
const static u8 azspio_null_component_id[AZSPIO_COMPONENT_ID_LENGTH];
#define azspio_system_component_id azspio_null_component_id

/* list of all nodes */
static LIST_HEAD(azspio_all_nodes);
/* lock for azspio_all_nodes and node reference */
static DEFINE_MUTEX(azspio_node_lock);

static LIST_HEAD(azspio_all_sockets);
static DEFINE_MUTEX(azspio_socket_lock);

struct azspio_allowed_communication {
	u8 src[AZSPIO_COMPONENT_ID_LENGTH];
	u8 dst[AZSPIO_COMPONENT_ID_LENGTH];
	struct list_head item;
};
/* list of all allowed communication pairs */
static LIST_HEAD(azspio_whitelist);
/* lock for azspio_whitelist */
static DEFINE_MUTEX(azspio_whitelist_lock);

atomic_long_t azspio_memory_allocated;
EXPORT_SYMBOL(azspio_memory_allocated);

/* defaults, but they don't matter much as azspio has a
 * more restrictive sndbuf/rcvbuf which are cgroup-accounted. */
static long sysctl_azspio_mem[3] = { 768 << 3, 1024 << 3, 1536 << 3 };
static int sysctl_azspio_wmem[3] = { 4 * 1024, 16 * 1024, 128 * 1024 };
static int sysctl_azspio_rmem[3] = { 4 * 1024, 87380, 87380 * 2 };

static bool azspio_id_matches(const void *id1, const void *id2)
{
	return !memcmp(id1, id2, AZSPIO_COMPONENT_ID_LENGTH);
}

static struct azspio_allowed_communication *azspio_find(const u8 *src,
							      const u8 *dst)
{
	// assumes we have a lock
	struct azspio_allowed_communication *allow;

	list_for_each_entry(allow, &azspio_whitelist, item)	{
		if (azspio_id_matches(allow->src, src) &&
			azspio_id_matches(allow->dst, dst))
		{
			return allow;
		}
	}
	return NULL;
}

static bool azspio_allow(const u8 *src, const u8 *dst)
{
	if (azspio_id_matches(src, azspio_system_component_id)) {
		return true;
	}

	return azspio_find(src, dst) != NULL;
}

static int azspio_allow_communication_set(const u8 *src, const u8 *dst,
					  int enable)
{
	int rc = 0;
	struct azspio_allowed_communication *allow, *next;
	bool src_null = azspio_id_matches(src, azspio_null_component_id);
	bool dst_null = azspio_id_matches(dst, azspio_null_component_id);
	if (!enable) {
		rc = -ENOENT;
		mutex_lock(&azspio_whitelist_lock);
		list_for_each_entry_safe(allow, next, &azspio_whitelist, item) {
			if ((src_null || azspio_id_matches(src, allow->src)) &&
			    (dst_null || azspio_id_matches(dst, allow->dst))) {

				list_del(&allow->item);
				kfree(allow);
				rc = 0;
			}
		}

		mutex_unlock(&azspio_whitelist_lock);

	} else {
		if (dst_null) {
			return -EINVAL;
		}

		mutex_lock(&azspio_whitelist_lock);
		allow = azspio_find(src, dst);
		if (!allow) {
			allow = kmalloc(sizeof(*allow), GFP_KERNEL);
			if (allow) {
				memcpy(allow->src, src, sizeof(allow->src));
				memcpy(allow->dst, dst, sizeof(allow->dst));
				list_add(&allow->item, &azspio_whitelist);
			} else {
				rc = -ENOMEM;
			}
		} else {
			rc = -EEXIST;
		}

		mutex_unlock(&azspio_whitelist_lock);
	}

	return rc;
}

static bool azspio_is_allowed(const u8 *src, const u8 *dst)
{
	bool result;

	mutex_lock(&azspio_whitelist_lock);
	result = azspio_allow(src, dst);
	mutex_unlock(&azspio_whitelist_lock);

	return result;
}

/**
 * struct azspio_node - endpoint node
 * @ep_lock: lock for endpoint management and callbacks
 * @ep: endpoint
 * @ref: reference count for node
 * @nid: node id
 * @item: list item for broadcast list
 */
struct azspio_node {
	struct mutex ep_lock;
	struct azspio_endpoint *ep;
	struct kref ref;
	u8 component_id[AZSPIO_COMPONENT_ID_LENGTH];

	struct sk_buff_head tx_queue;
	struct mutex tx_lock;

	struct list_head item;
};

/* Release node resources and free the node.
 *
 * Do not call directly, use azspio_node_release.  To be used with
 * kref_put_mutex.  As such, the node mutex is expected to be locked on call.
 */
static void __azspio_node_release(struct kref *kref)
{
	struct azspio_node *node = container_of(kref, struct azspio_node, ref);

	list_del(&node->item);
	mutex_unlock(&azspio_node_lock);

	skb_queue_purge(&node->tx_queue);
	kfree(node);
}

/* Increment reference to node. */
static struct azspio_node *azspio_node_acquire(struct azspio_node *node)
{
	if (node)
		kref_get(&node->ref);
	return node;
}

/* Decrement reference to node and release as necessary. */
static void azspio_node_release(struct azspio_node *node)
{
	if (!node)
		return;
	kref_put_mutex(&node->ref, __azspio_node_release, &azspio_node_lock);
}

/* Pass an outgoing packet socket buffer to the endpoint driver. */
static int azspio_node_enqueue(struct azspio_node *node, struct sk_buff *skb)
{
	int rc = -ENODEV;

	mutex_lock(&node->ep_lock);
	if (node->ep)
		rc = node->ep->xmit(node->ep, skb);
	else
		kfree_skb(skb);
	mutex_unlock(&node->ep_lock);

	return rc;
}

/* Pass an outgoing packet socket buffer to a socket */
static int azspio_socket_enqueue(struct azspio_sock *node, struct sk_buff *skb)
{
	int rc;
	rc = sock_queue_rcv_skb(&node->sk, skb);

	if (rc == -ENOBUFS || rc == -ENOMEM) {
		/* Don't free SKB since we'll backlog it */
		return rc;
	}

	if (rc)
		kfree_skb(skb);

	return rc;
}

/* Lookup node by component id.
 *
 * callers must release with azspio_node_release()
 */
static struct azspio_node *azspio_node_lookup(const u8 *component_id)
{
	struct azspio_node *res = 0, *node;

	mutex_lock(&azspio_node_lock);

	list_for_each_entry (node, &azspio_all_nodes, item) {
		if (azspio_id_matches(node->component_id, component_id)) {
			res = node;
			break;
		}
	}

	if (res) {
		azspio_node_acquire(res);
	}

	mutex_unlock(&azspio_node_lock);

	return res;
}

static struct azspio_sock *azspio_socket_lookup(const u8 *src, const u8 *dst);
static void azspio_socket_put(struct azspio_sock *ipc);

/**
 * azspio_endpoint_post() - post incoming data from nodes
 * @ep: endpoint handle
 * @skb: the skb; ownership is returned on error, remains in azspio for success.
 *
 * Return: 0 on success; negative error code on failure
 */
int azspio_endpoint_post(struct azspio_endpoint *ep, struct sk_buff *skb)
{
	struct azspio_node *node = ep->node;
	const struct azspio_hdr *phdr;
	struct azspio_sock *ipc;
	int rc;
	bool stop_rx = false;

	phdr = (const struct azspio_hdr *)skb_transport_header(skb);

	// Incoming packet from node

	if (!azspio_is_allowed(node->component_id, phdr->component_id)) {
		pr_warn_ratelimited("azspio_endpoint_post: incoming packet dropped because communication isn't allowed\n");
		return -EPERM;
	}

	mutex_lock(&azspio_socket_lock);
	ipc = azspio_socket_lookup(phdr->component_id, node->component_id);
	mutex_unlock(&azspio_socket_lock);

	if (!ipc) {
		pr_debug_ratelimited("azspio_endpoint_post: nobody listening; dropping\n");
		kfree_skb(skb);
		rc = 0;
	} else {
		if (ipc->throttled) {
			/* This should not happen because the socket is throttled,
			 * the endpoint should also be throttled and not produce
			 * new data. */
			pr_warn("azspio_endpoint_post: Socket already throttled; dropping.\n");
			rc = -ENOSPC;
		} else {
			rc = sock_queue_rcv_skb(&ipc->sk, skb);

			/* drop skb if it couldn't be enqueued */
			if (rc) {
				pr_warn_ratelimited("azspio_endpoint_post: packet dropped, rc=%d\n",
					rc);
			}

			/* When this socket would not receive further data, 
			 * mark as throttled, and globally stop receiving data. */
			if (atomic_read(&ipc->sk.sk_rmem_alloc) >=
			    ipc->sk.sk_rcvbuf) {
				ipc->throttled = true;
				stop_rx = true;
			}
		}
		azspio_socket_put(ipc);
	}

	if (stop_rx) {
		struct azspio_node *node;
		/* disable RX from all nodes */
		list_for_each_entry (node, &azspio_all_nodes, item) {
			node->ep->throttle(node->ep, true);
		}
	}

	return rc;
}

EXPORT_SYMBOL_GPL(azspio_endpoint_post);

/**
 * azspio_endpoint_notify() - Notify socket that outbound ringbuffer has capacity
 * @ep: endpoint
 *
 * Must be called from non-atomic context.
 */
void azspio_endpoint_notify(struct azspio_endpoint *ep)
{
	struct azspio_node *node = ep->node;
	struct sk_buff *skb;
	int rc;

	mutex_lock(&node->tx_lock);
	while ((skb = skb_dequeue(&node->tx_queue)) != NULL) {
		rc = azspio_node_enqueue(node, skb);
		if (rc == -ENOSPC) {
			// Only re-queue the packet on -ENOSPC.
			// The lower level consumes the skb on other errors.
			skb_queue_head(&node->tx_queue, skb);
		}

		if (rc < 0) {
			break;
		}
	}
	mutex_unlock(&node->tx_lock);
}

EXPORT_SYMBOL_GPL(azspio_endpoint_notify);

/**
 * azspio_endpoint_register() - register a new endpoint
 * @ep: endpoint to register
 * @component_id: The component ID of the endpoint to register.
 * Return: 0 on success; negative error code on failure
 *
 * The specified endpoint must have the xmit and throttle function pointer set on call.
 */
int azspio_endpoint_register(struct azspio_endpoint *ep, const u8 *component_id)
{
	struct azspio_node *node;

	if (!ep || !ep->xmit)
		return -EINVAL;

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return -ENOMEM;

	kref_init(&node->ref);
	mutex_init(&node->ep_lock);
	memcpy(node->component_id, component_id, sizeof(node->component_id));
	node->ep = ep;
	skb_queue_head_init(&node->tx_queue);
	mutex_init(&node->tx_lock);

	mutex_lock(&azspio_node_lock);
	list_add(&node->item, &azspio_all_nodes);
	mutex_unlock(&azspio_node_lock);
	ep->node = node;

	return 0;
}
EXPORT_SYMBOL_GPL(azspio_endpoint_register);

/**
 * azspio_endpoint_unregister - unregister endpoint
 * @ep: endpoint to unregister
 */
void azspio_endpoint_unregister(struct azspio_endpoint *ep)
{
	struct azspio_node *node = ep->node;

	mutex_lock(&node->ep_lock);
	node->ep = NULL;
	mutex_unlock(&node->ep_lock);

	azspio_node_release(node);
	ep->node = NULL;
}
EXPORT_SYMBOL_GPL(azspio_endpoint_unregister);

/* Lookup connected sockets with matching component_id
 *
 * Callers must release with azspio_socket_put()
 */
static struct azspio_sock *azspio_socket_lookup(const u8 *src, const u8 *dst)
{
	struct azspio_sock *ipc = NULL, *socket;

	list_for_each_entry (socket, &azspio_all_sockets, item) {
		if (socket->sk.sk_state == TCP_ESTABLISHED &&
		    azspio_id_matches(socket->us.sa_component_id, src) &&
		    azspio_id_matches(socket->peer.sa_component_id, dst)) {
			ipc = socket;
			break;
		}
	}

	if (ipc)
		sock_hold(&ipc->sk);

	return ipc;
}

/* Release acquired socket. */
static void azspio_socket_put(struct azspio_sock *ipc)
{
	sock_put(&ipc->sk);
}

/* Add socket to global socket list. */
static int azspio_socket_assign(struct azspio_sock *ipc)
{
	mutex_lock(&azspio_socket_lock);
	list_add(&ipc->item, &azspio_all_sockets);
	mutex_unlock(&azspio_socket_lock);
	sock_hold(&ipc->sk);
	return 0;
}

/* Remove socket. */
static void azspio_socket_remove(struct azspio_sock *ipc)
{
	__sock_put(&ipc->sk);
	mutex_lock(&azspio_socket_lock);
	list_del(&ipc->item);
	mutex_unlock(&azspio_socket_lock);
}

/* Bind socket to address.
 *
 * Socket should be locked upon call.
 */
static int __azspio_bind(struct socket *sock,
			 const struct sockaddr_azspio *addr, int zapped)
{
	struct azspio_sock *ipc = azspio_sk(sock->sk);
	const u8 *component_id = addr->sa_component_id;
	struct azure_sphere_guid current_component_id;
	struct sock *sk = sock->sk;
	int rc;

	if (!azure_sphere_get_component_id(&current_component_id, current)) {
		return -EINVAL;
	}

	if (!azspio_id_matches(component_id, &current_component_id)) {
		return -EACCES;
	}

	/* rebinding ok */
	if (!zapped && !azspio_id_matches(component_id, ipc->us.sa_component_id)) {
		return 0;
	}

	rc = azspio_socket_assign(ipc);
	if (rc)
		return rc;

	/* unbind previous, if any */
	if (!zapped)
		azspio_socket_remove(ipc);

	memcpy(ipc->us.sa_component_id, component_id,
		   sizeof(ipc->us.sa_component_id));

	if (azspio_id_matches(component_id, azspio_system_component_id)) {
		struct azspio_allowed_communication *allow = NULL;
		mutex_lock(&azspio_whitelist_lock);
		allow = kmalloc(sizeof(*allow), GFP_KERNEL);
		if (allow) {
			memcpy(allow->src, component_id, sizeof(allow->src));
			memcpy(allow->dst, component_id, sizeof(allow->dst));
			list_add(&allow->item, &azspio_whitelist);
		} else {
			rc = -ENOMEM;
		}

		mutex_unlock(&azspio_whitelist_lock);
	}

	if (!rc) {
		sock_reset_flag(sk, SOCK_ZAPPED);
	}

	return rc;
}

/* Auto bind to current component id. */
static int azspio_autobind(struct socket *sock)
{
	struct sock *sk = sock->sk;
	struct sockaddr_azspio addr;

	if (!sock_flag(sk, SOCK_ZAPPED))
		return 0;

	addr.sa_family = AF_AZSPIO;
	addr.sa_port = 0;

	if (!azure_sphere_get_component_id(
		    (struct azure_sphere_guid *)addr.sa_component_id,
		    current)) {
		pr_err("couldn't get current component ID from LSM\n");
		return -EINVAL;
	}

	return __azspio_bind(sock, &addr, 1);
}

/* Bind socket to specified sockaddr. */
static int azspio_bind(struct socket *sock, struct sockaddr *saddr, int len)
{
	DECLARE_SOCKADDR(struct sockaddr_azspio *, addr, saddr);
	struct azspio_sock *ipc = azspio_sk(sock->sk);
	struct sock *sk = sock->sk;
	int rc;

	if (len < sizeof(*addr) || addr->sa_family != AF_AZSPIO) {
		return -EINVAL;
	}

	if (addr->sa_port != 0) {
		return -EINVAL;
	}

	if (!azspio_id_matches(addr->sa_component_id, ipc->us.sa_component_id)) {
		return -EINVAL;
	}

	lock_sock(sk);
	rc = __azspio_bind(sock, addr, sock_flag(sk, SOCK_ZAPPED));
	release_sock(sk);

	return rc;
}

static int azspio_sendmsg(struct socket *sock, struct msghdr *msg, size_t len)
{
	DECLARE_SOCKADDR(struct sockaddr_azspio *, addr, msg->msg_name);
	struct azspio_sock *ipc = azspio_sk(sock->sk), *socket = NULL;
	struct sock *sk = sock->sk;
	struct azspio_node *node;
	struct azspio_hdr *hdr;
	struct sk_buff *skb;
	int rc;

	if (msg->msg_flags & ~(MSG_DONTWAIT))
		return -EINVAL;

	if (len > MTU)
		return -EMSGSIZE;

	lock_sock(sk);

	if (addr) {
		/* Address length must minimally be the expected length */
		if (msg->msg_namelen < sizeof(*addr)) {
			rc = -EINVAL;
			goto out_sock;
		}

		/* Family must be AF_AZSPIO */
		if (addr->sa_family != AF_AZSPIO) {
			rc = -EINVAL;
			goto out_sock;
		}

		/* Autobind to something so that return messages can be received */
		rc = azspio_autobind(sock);
		if (rc) {
			goto out_sock;
		}
	} else if (sk->sk_state == TCP_ESTABLISHED) {
		addr = &ipc->peer;
	} else {
		rc = -ENOTCONN;
		goto out_sock;
	}

	if (!azspio_is_allowed(ipc->us.sa_component_id,
			       addr->sa_component_id)) {
		pr_warn_ratelimited("azspio_sendmsg: communication not allowed\n");
		rc = -EPERM;
		goto out_sock;
	}

	/* Allocate an SKB for the data to send. This will block if the send queue is over capacity. */
	skb = sock_alloc_send_skb(sk, len + AZSPIO_HDR_SIZE,
				  msg->msg_flags & MSG_DONTWAIT, &rc);
	if (!skb)
		goto out_sock;

	skb_reset_transport_header(skb);
	skb_put(skb, len + AZSPIO_HDR_SIZE);

	hdr = (struct azspio_hdr *)skb_transport_header(skb);
	/* prefix packet with our address */
	memcpy(hdr->component_id, ipc->us.sa_component_id,
	       sizeof(hdr->component_id));

	/* Copy from userspace */
	rc = skb_copy_datagram_from_iter(skb, AZSPIO_HDR_SIZE, &msg->msg_iter,
					 len);
	if (rc)
		goto out_skb;

	/* check if the destination is a node */
	node = azspio_node_lookup(addr->sa_component_id);

	// otherwise see if it's a socket
	if (node == NULL) {
		mutex_lock(&azspio_socket_lock);
		socket = azspio_socket_lookup(addr->sa_component_id,
					      ipc->us.sa_component_id);
		mutex_unlock(&azspio_socket_lock);

		if (!socket) {
			rc = -ECONNRESET;
			goto out_skb;
		}
	}

	if (node != NULL) {
		/* Take tx_lock so that the queue full-ness doesn't change while 
		 * we decide to enqueue to ringbuffer vs. queue to tx_queue, 
		 * otherwise we may lose the notify */
		mutex_lock(&node->tx_lock);

		/* If there's something queued already, enforce FIFO by directly 
		 * enqueuing into the tx queue. */
		if (!skb_queue_empty(&node->tx_queue))
			rc = -ENOSPC;
		else
			rc = azspio_node_enqueue(node, skb);

		if (rc == -ENOSPC) {
			/* Note: skb wasn't released by azspio_node_enqueue for -ENOSPC */

			/* Enqueue for this node's tx queue */
			skb_queue_tail(&node->tx_queue, skb);
			rc = 0;
		}

		mutex_unlock(&node->tx_lock);

		azspio_node_release(node);
	} else if (socket != NULL) {
		/* If there's something in the backlog, enforce FIFO by enqueuing into
		 * the backlog. */
		if (!skb_queue_empty(&socket->rx_backlog))
			rc = -ENOMEM;
		else
			rc = azspio_socket_enqueue(socket, skb);

		if (rc == -ENOMEM || rc == -ENOBUFS) {
			/* Note: skb wasn't released in azspio_socket_enqueue */

			/* Enqueue for the dest socket backlog */
			skb_queue_tail(&socket->rx_backlog, skb);
			rc = 0;
		}
		azspio_socket_put(socket);
	}

	if (rc >= 0)
		rc = len;

	/* the skb was either sent or queued. */
	goto out_sock;

out_skb:
	kfree_skb(skb);
out_sock:
	release_sock(sk);

	return rc;
}

static int azspio_recvmsg(struct socket *sock, struct msghdr *msg, size_t size,
			  int flags)
{
	DECLARE_SOCKADDR(struct sockaddr_azspio *, addr, msg->msg_name);
	const struct azspio_hdr *phdr;
	struct azspio_sock *ipc = azspio_sk(sock->sk);
	struct sock *sk = sock->sk;
	struct sk_buff *skb;
	int copied, rc, rc_backlog;
	bool unthrottle = false;

	lock_sock(sk);

	if (sock_flag(sk, SOCK_ZAPPED)) {
		release_sock(sk);
		return -EADDRNOTAVAIL;
	}

	/* Obtain skb from receive queue; this conditionally blocks 
	 *and handles peek etc. */
	skb = skb_recv_datagram(sk, flags & ~MSG_DONTWAIT, flags & MSG_DONTWAIT,
				&rc);
	if (!skb) {
		release_sock(sk);
		return rc;
	}

	/* If this socket was previously causing the nodes to be throttled, and 
	 * de-queueing caused it to have enough space for more data, attempt to 
	 * un-throttle at the end. */
	if (ipc->throttled && atomic_read(&sk->sk_rmem_alloc) < sk->sk_rcvbuf) {
		unthrottle = true;
		ipc->throttled = false;
	}

	copied = skb->len - AZSPIO_HDR_SIZE;

	if (copied > size) {
		copied = size;
		msg->msg_flags |= MSG_TRUNC;
	}

	rc = skb_copy_datagram_msg(skb, AZSPIO_HDR_SIZE, msg, copied);
	if (rc < 0) {
		goto out;
	}
	rc = copied;

	if (addr) {
		addr->sa_family = AF_AZSPIO;

		/* Obtain address from header */
		phdr = (const struct azspio_hdr *)skb_transport_header(skb);
		memcpy(addr->sa_component_id, phdr->component_id,
		       sizeof(addr->sa_component_id));
		msg->msg_namelen = sizeof(*addr);
	}

out:
	skb_free_datagram(sk, skb);

	/* If there are any, and there's free space, queue back packets from backlog */
	while (atomic_read(&sk->sk_rmem_alloc) < sk->sk_rcvbuf &&
	       (skb = skb_dequeue(&ipc->rx_backlog)) != NULL) {
		rc_backlog = sock_queue_rcv_skb(sk, skb);
		if (rc_backlog) {
			pr_err("azspio_recvmsg: sock_queue_rcv_skb rc=%d\n",
			       rc_backlog);
			/* put it back into the queue */
			skb_queue_head(&ipc->rx_backlog, skb);
			break;
		}
	}

	release_sock(sk);

	if (unthrottle) {
		/* re-enable RX for all */
		struct azspio_node *node;
		list_for_each_entry (node, &azspio_all_nodes, item) {
			node->ep->throttle(node->ep, false);
		}
	}

	return rc;
}

static int azspio_connect(struct socket *sock, struct sockaddr *saddr, int len,
			  int flags)
{
	DECLARE_SOCKADDR(struct sockaddr_azspio *, addr, saddr);
	struct azspio_sock *ipc = azspio_sk(sock->sk), *exist_ipc;
	struct sock *sk = sock->sk;
	int rc;

	if (len < sizeof(*addr) || addr->sa_family != AF_AZSPIO ||
	    addr->sa_port != 0) {
		return -EINVAL;
	}

	lock_sock(sk);

	sk->sk_state = TCP_CLOSE;
	sock->state = SS_UNCONNECTED;

	rc = azspio_autobind(sock);
	if (rc) {
		release_sock(sk);
		return rc;
	}

	/* We don't allow a second socket with the same connection */
	mutex_lock(&azspio_socket_lock);
	exist_ipc = azspio_socket_lookup(ipc->us.sa_component_id,
					 addr->sa_component_id);
	if (exist_ipc) {
		mutex_unlock(&azspio_socket_lock);
		azspio_socket_put(exist_ipc);
		release_sock(sk);
		return -EADDRINUSE;
	}

	/* Check that communication is allowed to this peer */
	if (!azspio_is_allowed(ipc->us.sa_component_id,
				addr->sa_component_id) &&
			/* Allow clients to connect to the system socket (but not send data) */
			!azspio_id_matches(addr->sa_component_id, azspio_system_component_id)) {
		mutex_unlock(&azspio_socket_lock);
		release_sock(sk);
		return -EPERM;
	}

	ipc->peer = *addr;
	sock->state = SS_CONNECTED;
	sk->sk_state = TCP_ESTABLISHED;
	mutex_unlock(&azspio_socket_lock);

	release_sock(sk);

	return 0;
}

static int azspio_getname(struct socket *sock, struct sockaddr *saddr, int *len,
			  int peer)
{
	struct azspio_sock *ipc = azspio_sk(sock->sk);
	struct sockaddr_azspio qaddr;
	struct sock *sk = sock->sk;

	lock_sock(sk);
	if (peer) {
		if (sk->sk_state != TCP_ESTABLISHED) {
			release_sock(sk);
			return -ENOTCONN;
		}

		qaddr = ipc->peer;
	} else {
		qaddr = ipc->us;
	}
	release_sock(sk);

	*len = sizeof(qaddr);
	qaddr.sa_family = AF_AZSPIO;

	memcpy(saddr, &qaddr, sizeof(qaddr));

	return 0;
}

static int azspio_ioctl(struct socket *sock, unsigned int cmd,
			unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct azspio_sock *ipc = azspio_sk(sock->sk);
	struct sock *sk = sock->sk;
	struct sockaddr_azspio *sq;
	struct ifreq ifr;
	long len = 0;
	int rc = 0;

	lock_sock(sk);

	switch (cmd) {
	case TIOCOUTQ:
		/* Get send queue depth, i.e. data not yet in the ringbuffer */
		len = sk->sk_sndbuf - sk_wmem_alloc_get(sk);
		if (len < 0)
			len = 0;
		rc = put_user(len, (int __user *)argp);
		break;
	case TIOCINQ:
		/* Get receive queue depth */
		len = sk->sk_rcvbuf - sk_rmem_alloc_get(sk);
		if (len < 0)
			len = 0;
		rc = put_user(len, (int __user *)argp);
		break;
	case SIOCGIFADDR:
		/* Allows to get "our" address, i.e. current component ID */
		if (copy_from_user(&ifr, argp, sizeof(ifr))) {
			rc = -EFAULT;
			break;
		}

		sq = (struct sockaddr_azspio *)&ifr.ifr_addr;
		*sq = ipc->us;
		if (copy_to_user(argp, &ifr, sizeof(ifr))) {
			rc = -EFAULT;
			break;
		}
		break;
	case SIOCGSTAMP:
		/* Timestamp of last received (via recvmsg) packet */
		rc = sock_get_timestamp(sk, argp);
		break;
	case SIOCAZSPIOFWADD:
	case SIOCAZSPIOFWDEL: {
		struct azspio_fw_entry entry;
		bool enable;

		if (!capable(CAP_NET_ADMIN)) {
			rc = -EPERM;
			break;
		}

		if (cmd == SIOCAZSPIOFWADD)
			enable = 1;
		else
			enable = 0;

		if (copy_from_user(&entry, argp, sizeof(entry))) {
			rc = -EFAULT;
			break;
		}

		if (entry.src.sa_family != AF_AZSPIO ||
		    entry.dst.sa_family != AF_AZSPIO ||
		    entry.src.sa_port != 0 || entry.dst.sa_port != 0) {
			rc = -EINVAL;
			break;
		}

		rc = azspio_allow_communication_set(entry.src.sa_component_id,
						    entry.dst.sa_component_id,
						    enable);
		break;
	}
	default:
		rc = -ENOIOCTLCMD;
		break;
	}

	release_sock(sk);

	return rc;
}

static int azspio_release(struct socket *sock)
{
	struct sock *sk = sock->sk;
	struct azspio_sock *ipc;

	if (!sk)
		return 0;

	lock_sock(sk);

	ipc = azspio_sk(sk);
	sk->sk_shutdown = SHUTDOWN_MASK;
	if (!sock_flag(sk, SOCK_DEAD))
		sk->sk_state_change(sk);

	sock_set_flag(sk, SOCK_DEAD);

	sock_orphan(sk);
	sock->sk = NULL;

	if (!sock_flag(sk, SOCK_ZAPPED))
		azspio_socket_remove(ipc);

	skb_queue_purge(&sk->sk_receive_queue);
	skb_queue_purge(&ipc->rx_backlog);

	sk_mem_reclaim(sk);

	release_sock(sk);
	sock_put(sk);

	return 0;
}

static const struct proto_ops azspio_proto_ops = {
	.owner = THIS_MODULE,
	.family = AF_AZSPIO,
	.bind = azspio_bind,
	.connect = azspio_connect,
	.socketpair = sock_no_socketpair,
	.accept = sock_no_accept,
	.listen = sock_no_listen,
	.sendmsg = azspio_sendmsg,
	.recvmsg = azspio_recvmsg,
	.getname = azspio_getname,
	.ioctl = azspio_ioctl,
	.poll = datagram_poll,
	.shutdown = sock_no_shutdown,
	.setsockopt = sock_no_setsockopt,
	.getsockopt = sock_no_getsockopt,
	.release = azspio_release,
	.mmap = sock_no_mmap,
	.sendpage = sock_no_sendpage,
};

static struct proto azspio_proto = {
	.name = "AZSPIO",
	.owner = THIS_MODULE,
	.memory_allocated = &azspio_memory_allocated,
	.sysctl_mem = sysctl_azspio_mem,
	.sysctl_wmem = sysctl_azspio_wmem,
	.sysctl_rmem = sysctl_azspio_rmem,
	.obj_size = sizeof(struct azspio_sock),
};

static int azspio_create(struct net *net, struct socket *sock, int protocol,
			 int kern)
{
	struct azspio_sock *ipc;
	struct sock *sk;

	if (sock->type != SOCK_DGRAM)
		return -EPROTOTYPE;

	sk = sk_alloc(net, AF_AZSPIO, GFP_KERNEL, &azspio_proto, kern);
	if (!sk)
		return -ENOMEM;

	sock_set_flag(sk, SOCK_ZAPPED);

	sock_init_data(sock, sk);
	/* Set default send and receive queue to 16384 bytes. This is on top of
	 * ringbuffers. */
	sk->sk_rcvbuf = 16384;
	sk->sk_sndbuf = 16384;
	sock->ops = &azspio_proto_ops;

	ipc = azspio_sk(sk);
	ipc->us.sa_family = AF_AZSPIO;

	skb_queue_head_init(&ipc->rx_backlog);
	mutex_init(&ipc->rx_backlog_lock);

	/* Fill local address from component id of caller */
	if (!azure_sphere_get_component_id(
		    (struct azure_sphere_guid *)ipc->us.sa_component_id,
		    current)) {
		sk_free(sk);
		return -EINVAL;
	}

	return 0;
}

static const struct net_proto_family azspio_family = {
	.owner = THIS_MODULE,
	.family = AF_AZSPIO,
	.create = azspio_create,
};

static int __init azspio_proto_init(void)
{
	int rc;

	rc = proto_register(&azspio_proto, 1);
	if (rc)
		return rc;

	rc = sock_register(&azspio_family);
	if (rc) {
		proto_unregister(&azspio_proto);
		return rc;
	}

	return 0;
}
module_init(azspio_proto_init);

static void __exit azspio_proto_fini(void)
{
	sock_unregister(azspio_family.family);
	proto_unregister(&azspio_proto);
}
module_exit(azspio_proto_fini);

MODULE_DESCRIPTION("Azure Sphere IPC");
MODULE_LICENSE("GPL v2");
