// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 Microsoft Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place - Suite 330, Boston, MA 02111-1307 USA.
 *
 */
#ifndef __AZSPIO_H_
#define __AZSPIO_H_

#include <linux/types.h>

struct sk_buff;

/**
 * struct azspio_endpoint - endpoint handle
 * @xmit: Callback for outgoing packets
 * The socket buffer passed to the xmit function becomes owned by the endpoint
 * driver.  As such, when the driver is done with the buffer, it should
 * call kfree_skb() on failure, or consume_skb() on success.
 * The exception is -ENOSPC; in this case, the skb will not be owned by the 
 * endpoint (since the socket will attempt to re-transmit).
 * @throttle: Set RX throttling
 * As a response to azspio_endpoint_post, if the queue is filled up so that no
 * more packets can be enqueued, throttle is called with throttled=true.
 * When though data has been removed, throttle is called with throttled=false.
 * Note that _all_ endpoints will be throttled equally if a single socket
 * recvqueue is over capacity.
 */
struct azspio_endpoint {
	int (*xmit)(struct azspio_endpoint *ep, struct sk_buff *skb);
	void (*throttle)(struct azspio_endpoint *ep, bool throttled);
	/* private: not for endpoint use */
	struct azspio_node *node;
};

int azspio_endpoint_register(struct azspio_endpoint *ep,
			     const u8 *component_id);
void azspio_endpoint_unregister(struct azspio_endpoint *ep);
int azspio_endpoint_post(struct azspio_endpoint *ep, struct sk_buff *skb);
void azspio_endpoint_notify(struct azspio_endpoint *ep);

#endif
