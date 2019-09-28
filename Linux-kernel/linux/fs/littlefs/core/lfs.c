/*
 * The little filesystem
 *
 * Copyright (c) 2017, Arm Limited. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "lfs.h"
#include "lfs_util.h"


/// Caching block device operations ///
static int lfs_cache_read(lfs_t *lfs, lfs_cache_t *rcache,
        const lfs_cache_t *pcache, lfs_block_t block,
        lfs_off_t off, void *buffer, lfs_size_t size) {
    uint8_t *data = buffer;
    int err;
    LFS_ASSERT(block != 0xffffffff);

    while (size > 0) {
        if (pcache && block == pcache->block && off >= pcache->off &&
                off < pcache->off + lfs->cfg->prog_size) {
            // is already in pcache?
            lfs_size_t diff = lfs_min(size,
                    lfs->cfg->prog_size - (off-pcache->off));
            memcpy(data, &pcache->buffer[off-pcache->off], diff);

            data += diff;
            off += diff;
            size -= diff;
            continue;
        }

        if (block == rcache->block && off >= rcache->off &&
                off < rcache->off + lfs->cfg->read_size) {
            // is already in rcache?
            lfs_size_t diff = lfs_min(size,
                    lfs->cfg->read_size - (off-rcache->off));
            memcpy(data, &rcache->buffer[off-rcache->off], diff);

            data += diff;
            off += diff;
            size -= diff;
            continue;
        }

        if (off % lfs->cfg->read_size == 0 && size >= lfs->cfg->read_size) {
            // bypass cache?
            lfs_size_t diff = size - (size % lfs->cfg->read_size);
            int err = lfs->cfg->read(lfs->cfg, block, off, data, diff);
            if (err) {
                return err;
            }

            data += diff;
            off += diff;
            size -= diff;
            continue;
        }

        // load to cache, first condition can no longer fail
        LFS_ASSERT(block < lfs->cfg->block_count);
        rcache->block = block;
        rcache->off = off - (off % lfs->cfg->read_size);
        err = lfs->cfg->read(lfs->cfg, rcache->block,
                rcache->off, rcache->buffer, lfs->cfg->read_size);
        if (err) {
            return err;
        }
    }

    return 0;
}

static int lfs_cache_cmp(lfs_t *lfs, lfs_cache_t *rcache,
        const lfs_cache_t *pcache, lfs_block_t block,
        lfs_off_t off, const void *buffer, lfs_size_t size) {
    const uint8_t *data = buffer;
    lfs_off_t i;

    for (i = 0; i < size; i++) {
        uint8_t c;
        int err = lfs_cache_read(lfs, rcache, pcache,
                block, off+i, &c, 1);
        if (err) {
            return err;
        }

        if (c != data[i]) {
            return false;
        }
    }

    return true;
}

static int lfs_cache_crc(lfs_t *lfs, lfs_cache_t *rcache,
        const lfs_cache_t *pcache, lfs_block_t block,
        lfs_off_t off, lfs_size_t size, uint32_t *crc) {
    lfs_off_t i;
    for (i = 0; i < size; i++) {
        uint8_t c;
        int err = lfs_cache_read(lfs, rcache, pcache,
                block, off+i, &c, 1);
        if (err) {
            return err;
        }

        lfs_crc(crc, &c, 1);
    }

    return 0;
}

static int lfs_cache_flush(lfs_t *lfs,
        lfs_cache_t *pcache, lfs_cache_t *rcache) {
    int err;
    if (pcache->block != 0xffffffff) {
        LFS_ASSERT(pcache->block < lfs->cfg->block_count);
        err = lfs->cfg->prog(lfs->cfg, pcache->block,
                pcache->off, pcache->buffer, lfs->cfg->prog_size);
        if (err) {
            return err;
        }

        if (rcache) {
            int res = lfs_cache_cmp(lfs, rcache, NULL, pcache->block,
                    pcache->off, pcache->buffer, lfs->cfg->prog_size);
            if (res < 0) {
                return res;
            }

            if (!res) {
                return LFS_ERR_CORRUPT;
            }
        }

        pcache->block = 0xffffffff;
    }

    return 0;
}

static int lfs_cache_prog(lfs_t *lfs, lfs_cache_t *pcache,
        lfs_cache_t *rcache, lfs_block_t block,
        lfs_off_t off, const void *buffer, lfs_size_t size) {
    const uint8_t *data = buffer;
    LFS_ASSERT(block != 0xffffffff);
    LFS_ASSERT(off + size <= lfs->cfg->block_size);

    while (size > 0) {
        if (block == pcache->block && off >= pcache->off &&
                off < pcache->off + lfs->cfg->prog_size) {
            // is already in pcache?
            lfs_size_t diff = lfs_min(size,
                    lfs->cfg->prog_size - (off-pcache->off));
            memcpy(&pcache->buffer[off-pcache->off], data, diff);

            data += diff;
            off += diff;
            size -= diff;

            if (off % lfs->cfg->prog_size == 0) {
                // eagerly flush out pcache if we fill up
                int err = lfs_cache_flush(lfs, pcache, rcache);
                if (err) {
                    return err;
                }
            }

            continue;
        }

        // pcache must have been flushed, either by programming and
        // entire block or manually flushing the pcache
        LFS_ASSERT(pcache->block == 0xffffffff);

        if (off % lfs->cfg->prog_size == 0 &&
                size >= lfs->cfg->prog_size) {
            int err;
            lfs_size_t diff;

            // bypass pcache?
            LFS_ASSERT(block < lfs->cfg->block_count);
            diff = size - (size % lfs->cfg->prog_size);
            err = lfs->cfg->prog(lfs->cfg, block, off, data, diff);
            if (err) {
                return err;
            }

            if (rcache) {
                int res = lfs_cache_cmp(lfs, rcache, NULL,
                        block, off, data, diff);
                if (res < 0) {
                    return res;
                }

                if (!res) {
                    return LFS_ERR_CORRUPT;
                }
            }

            data += diff;
            off += diff;
            size -= diff;
            continue;
        }

        // prepare pcache, first condition can no longer fail
        pcache->block = block;
        pcache->off = off - (off % lfs->cfg->prog_size);
    }

    return 0;
}


/// General lfs block device operations ///
static int lfs_bd_read(lfs_t *lfs, lfs_block_t block,
        lfs_off_t off, void *buffer, lfs_size_t size) {
    // if we ever do more than writes to alternating pairs,
    // this may need to consider pcache
    return lfs_cache_read(lfs, &lfs->rcache, NULL,
            block, off, buffer, size);
}

static int lfs_bd_prog(lfs_t *lfs, lfs_block_t block,
        lfs_off_t off, const void *buffer, lfs_size_t size) {
    return lfs_cache_prog(lfs, &lfs->pcache, NULL,
            block, off, buffer, size);
}

static int lfs_bd_cmp(lfs_t *lfs, lfs_block_t block,
        lfs_off_t off, const void *buffer, lfs_size_t size) {
    return lfs_cache_cmp(lfs, &lfs->rcache, NULL, block, off, buffer, size);
}

static int lfs_bd_crc(lfs_t *lfs, lfs_block_t block,
        lfs_off_t off, lfs_size_t size, uint32_t *crc) {
    return lfs_cache_crc(lfs, &lfs->rcache, NULL, block, off, size, crc);
}

static int lfs_bd_erase(lfs_t *lfs, lfs_block_t block) {
    LFS_ASSERT(block < lfs->cfg->block_count);
    return lfs->cfg->erase(lfs->cfg, block);
}

static int lfs_bd_sync(lfs_t *lfs) {
    int err;

    lfs->rcache.block = 0xffffffff;

    err = lfs_cache_flush(lfs, &lfs->pcache, NULL);
    if (err) {
        return err;
    }

    return lfs->cfg->sync(lfs->cfg);
}


/// Internal operations predeclared here ///
static int lfs_pred(lfs_t *lfs, const lfs_block_t dir[2], lfs_dir_t *pdir);
static int lfs_parent(lfs_t *lfs, const lfs_block_t dir[2],
        lfs_dir_t *parent, lfs_entry_t *entry);
static int lfs_moved(lfs_t *lfs, const void *e);
static int lfs_relocate(lfs_t *lfs,
        const lfs_block_t oldpair[2], const lfs_block_t newpair[2]);
int lfs_deorphan(lfs_t *lfs);


/// Block allocator ///
static int lfs_alloc_lookahead(void *p, lfs_block_t block) {
    lfs_t *lfs = p;

    lfs_block_t off = ((block - lfs->free.off)
            + lfs->cfg->block_count) % lfs->cfg->block_count;

    if (off < lfs->free.size) {
        lfs->free.buffer[off / 32] |= 1U << (off % 32);
    }

    return 0;
}

static int lfs_alloc(lfs_t *lfs, lfs_block_t *block) {
    int err;
    struct lfs_traverse_callbacks callbacks = {
        .block_callback = lfs_alloc_lookahead,
        .entry_callback = NULL
    };

    while (true) {
        while (lfs->free.i != lfs->free.size) {
            lfs_block_t off = lfs->free.i;
            lfs->free.i += 1;
            lfs->free.ack -= 1;

            if (!(lfs->free.buffer[off / 32] & (1U << (off % 32)))) {
                // found a free block
                *block = (lfs->free.off + off) % lfs->cfg->block_count;

                // eagerly find next off so an alloc ack can
                // discredit old lookahead blocks
                while (lfs->free.i != lfs->free.size &&
                        (lfs->free.buffer[lfs->free.i / 32]
                            & (1U << (lfs->free.i % 32)))) {
                    lfs->free.i += 1;
                    lfs->free.ack -= 1;
                }

                return 0;
            }
        }

        // check if we have looked at all blocks since last ack
        if (lfs->free.ack == 0) {
            LFS_WARN("No more free space %d", lfs->free.i + lfs->free.off);
            return LFS_ERR_NOSPC;
        }

        lfs->free.off = (lfs->free.off + lfs->free.size)
                % lfs->cfg->block_count;
        lfs->free.size = lfs_min(lfs->cfg->lookahead, lfs->free.ack);
        lfs->free.i = 0;

        // find mask of free blocks from tree
        memset(lfs->free.buffer, 0, lfs->cfg->lookahead/8);
        err = lfs_traverse(lfs, callbacks, lfs);
        if (err) {
            return err;
        }
    }
}

static void lfs_alloc_ack(lfs_t *lfs) {
    lfs->free.ack = lfs->cfg->block_count;
}


/// Endian swapping functions ///
static void lfs_dir_fromle32(struct lfs_disk_dir *d) {
    d->rev     = lfs_fromle32(d->rev);
    d->size    = lfs_fromle32(d->size);
    d->tail[0] = lfs_fromle32(d->tail[0]);
    d->tail[1] = lfs_fromle32(d->tail[1]);
}

static void lfs_dir_tole32(struct lfs_disk_dir *d) {
    d->rev     = lfs_tole32(d->rev);
    d->size    = lfs_tole32(d->size);
    d->tail[0] = lfs_tole32(d->tail[0]);
    d->tail[1] = lfs_tole32(d->tail[1]);
}

static void lfs_entry_fromle32(struct lfs_disk_entry *d) {
    d->u.dir[0] = lfs_fromle32(d->u.dir[0]);
    d->u.dir[1] = lfs_fromle32(d->u.dir[1]);
}

static void lfs_entry_tole32(struct lfs_disk_entry *d) {
    d->u.dir[0] = lfs_tole32(d->u.dir[0]);
    d->u.dir[1] = lfs_tole32(d->u.dir[1]);
}

static void lfs_superblock_fromle32(struct lfs_disk_superblock *d) {
    d->root[0]     = lfs_fromle32(d->root[0]);
    d->root[1]     = lfs_fromle32(d->root[1]);
    d->block_size  = lfs_fromle32(d->block_size);
    d->block_count = lfs_fromle32(d->block_count);
    d->version     = lfs_fromle32(d->version);
    d->inline_size = lfs_fromle32(d->inline_size);
    d->attrs_size  = lfs_fromle32(d->attrs_size);
    d->name_size   = lfs_fromle32(d->name_size);
}

static void lfs_superblock_tole32(struct lfs_disk_superblock *d) {
    d->root[0]     = lfs_tole32(d->root[0]);
    d->root[1]     = lfs_tole32(d->root[1]);
    d->block_size  = lfs_tole32(d->block_size);
    d->block_count = lfs_tole32(d->block_count);
    d->version     = lfs_tole32(d->version);
    d->inline_size = lfs_tole32(d->inline_size);
    d->attrs_size  = lfs_tole32(d->attrs_size);
    d->name_size   = lfs_tole32(d->name_size);
}

/// Other struct functions ///
static inline lfs_size_t lfs_entry_elen(const lfs_entry_t *entry) {
    return (lfs_size_t)(entry->d.elen) |
        ((lfs_size_t)(entry->d.alen & 0xc0) << 2);
}

static inline lfs_size_t lfs_entry_alen(const lfs_entry_t *entry) {
    return entry->d.alen & 0x3f;
}

static inline lfs_size_t lfs_entry_nlen(const lfs_entry_t *entry) {
    return entry->d.nlen;
}

static inline lfs_size_t lfs_entry_size(const lfs_entry_t *entry) {
    return 4 + lfs_entry_elen(entry) +
            lfs_entry_alen(entry) +
            lfs_entry_nlen(entry);
}


/// Metadata pair and directory operations ///
static inline void lfs_pairswap(lfs_block_t pair[2]) {
    lfs_block_t t = pair[0];
    pair[0] = pair[1];
    pair[1] = t;
}

static inline bool lfs_pairisnull(const lfs_block_t pair[2]) {
    return pair[0] == 0xffffffff || pair[1] == 0xffffffff;
}

static inline int lfs_paircmp(
        const lfs_block_t paira[2],
        const lfs_block_t pairb[2]) {
    return !(paira[0] == pairb[0] || paira[1] == pairb[1] ||
             paira[0] == pairb[1] || paira[1] == pairb[0]);
}

static inline bool lfs_pairsync(
        const lfs_block_t paira[2],
        const lfs_block_t pairb[2]) {
    return (paira[0] == pairb[0] && paira[1] == pairb[1]) ||
           (paira[0] == pairb[1] && paira[1] == pairb[0]);
}

static int lfs_dir_alloc(lfs_t *lfs, lfs_dir_t *dir) {
    // allocate pair of dir blocks
    int i, err;
    for (i = 0; i < 2; i++) {
        int err = lfs_alloc(lfs, &dir->pair[i]);
        if (err) {
            return err;
        }
    }

    // rather than clobbering one of the blocks we just pretend
    // the revision may be valid
    err = lfs_bd_read(lfs, dir->pair[0], 0, &dir->d.rev, 4);
    dir->d.rev = lfs_fromle32(dir->d.rev);
    if (err) {
        return err;
    }

    // set defaults
    dir->d.rev += 1;
    dir->d.size = sizeof(dir->d)+4;
    dir->d.tail[0] = 0xffffffff;
    dir->d.tail[1] = 0xffffffff;
    dir->off = sizeof(dir->d);

    // don't write out yet, let caller take care of that
    return 0;
}

static int lfs_dir_fetch(lfs_t *lfs,
        lfs_dir_t *dir, const lfs_block_t pair[2]) {
    // copy out pair, otherwise may be aliasing dir
    const lfs_block_t tpair[2] = {pair[0], pair[1]};
    bool valid = false;
    int i;
    uint32_t crc;

    // check both blocks for the most recent revision
    for (i = 0; i < 2; i++) {
        struct lfs_disk_dir test;
        int err = lfs_bd_read(lfs, tpair[i], 0, &test, sizeof(test));
        lfs_dir_fromle32(&test);
        if (err) {
            return err;
        }

        if (valid && lfs_scmp(test.rev, dir->d.rev) < 0) {
            continue;
        }

        if ((0x7fffffff & test.size) < sizeof(test)+4 ||
            (0x7fffffff & test.size) > lfs->cfg->block_size) {
            continue;
        }

        crc = 0xffffffff;
        lfs_dir_tole32(&test);
        lfs_crc(&crc, &test, sizeof(test));
        lfs_dir_fromle32(&test);
        err = lfs_bd_crc(lfs, tpair[i], sizeof(test),
                (0x7fffffff & test.size) - sizeof(test), &crc);
        if (err) {
            return err;
        }

        if (crc != 0) {
            continue;
        }

        valid = true;

        // setup dir in case it's valid
        dir->pair[0] = tpair[(i+0) % 2];
        dir->pair[1] = tpair[(i+1) % 2];
        dir->off = sizeof(dir->d);
        dir->d = test;
    }

    if (!valid) {
        LFS_ERROR("Corrupted dir pair at %d %d", tpair[0], tpair[1]);
        return LFS_ERR_CORRUPT;
    }

    return 0;
}

struct lfs_region {
    enum {
        LFS_FROM_MEM,
        LFS_FROM_REGION,
        LFS_FROM_ATTRS,
    } type;

    lfs_off_t oldoff;
    lfs_size_t oldsize;
    const void *buffer;
    lfs_size_t newsize;
};

struct lfs_region_attrs {
    const struct lfs_attr *attrs;
    int count;
};

struct lfs_region_region {
    lfs_block_t block;
    lfs_off_t off;
    struct lfs_region *regions;
    int count;
};

static int lfs_commit_region(lfs_t *lfs, uint32_t *crc,
        lfs_block_t oldblock, lfs_off_t oldoff,
        lfs_block_t newblock, lfs_off_t newoff,
        lfs_off_t regionoff, lfs_size_t regionsize,
        const struct lfs_region *regions, int count) {
    int i = 0, err;
    int j;
    lfs_size_t newend = newoff + regionsize;
    while (newoff < newend) {
        // commit from different types of regions
        if (i < count && regions[i].oldoff == oldoff - regionoff) {
            switch (regions[i].type) {
                case LFS_FROM_MEM: {
                    lfs_crc(crc, regions[i].buffer, regions[i].newsize);
                    err = lfs_bd_prog(lfs, newblock, newoff,
                            regions[i].buffer, regions[i].newsize);
                    if (err) {
                        return err;
                    }
                    newoff += regions[i].newsize;
                    oldoff += regions[i].oldsize;
                    break;
                }
                case LFS_FROM_REGION: {
                    const struct lfs_region_region *disk = regions[i].buffer;
                    err = lfs_commit_region(lfs, crc,
                            disk->block, disk->off,
                            newblock, newoff,
                            disk->off, regions[i].newsize,
                            disk->regions, disk->count);
                    if (err) {
                        return err;
                    }
                    newoff += regions[i].newsize;
                    oldoff -= regions[i].oldsize;
                    break;
                }
                case LFS_FROM_ATTRS: {
                    lfs_off_t oldend;
                    const struct lfs_region_attrs *attrs = regions[i].buffer;

                    // order doesn't matter, so we write new attrs first. this
                    // is still O(n^2) but only O(n) disk access
                    for (j = 0; j < attrs->count; j++) {
                        lfs_entry_attr_t attr;

                        if (attrs->attrs[j].size == 0) {
                            continue;
                        }

                        attr.d.type = attrs->attrs[j].type;
                        attr.d.len = attrs->attrs[j].size;

                        lfs_crc(crc, &attr.d, sizeof(attr.d));
                        err = lfs_bd_prog(lfs, newblock, newoff,
                                &attr.d, sizeof(attr.d));
                        if (err) {
                            return err;
                        }

                        lfs_crc(crc,
                                attrs->attrs[j].buffer, attrs->attrs[j].size);
                        err = lfs_bd_prog(lfs, newblock, newoff+sizeof(attr.d),
                                attrs->attrs[j].buffer, attrs->attrs[j].size);
                        if (err) {
                            return err;
                        }

                        newoff += 2+attrs->attrs[j].size;
                    }

                    // copy over attributes without updates
                    oldend = oldoff + regions[i].oldsize;
                    while (oldoff < oldend) {
                        bool updating = false;
                        lfs_entry_attr_t attr;
                        err = lfs_bd_read(lfs, oldblock, oldoff,
                                &attr.d, sizeof(attr.d));
                        if (err) {
                            return err;
                        }

                        for (j = 0; j < attrs->count; j++) {
                            if (attr.d.type == attrs->attrs[j].type) {
                                updating = true;
                            }
                        }

                        if (!updating) {
                            err = lfs_commit_region(lfs, crc,
                                    oldblock, oldoff,
                                    newblock, newoff,
                                    0, 2+attr.d.len,
                                    NULL, 0);
                            if (err) {
                                return err;
                            }

                            newoff += 2+attr.d.len;
                        }

                        oldoff += 2+attr.d.len;
                    }

                    break;
                }
            }

            i += 1;
        } else {
            // copy data from old block if not covered by region
            uint8_t data;
            err = lfs_bd_read(lfs, oldblock, oldoff, &data, 1);
            if (err) {
                return err;
            }

            lfs_crc(crc, &data, 1);
            err = lfs_bd_prog(lfs, newblock, newoff, &data, 1);
            if (err) {
                return err;
            }

            oldoff += 1;
            newoff += 1;
        }
    }

    // sanity check our commit math
    LFS_ASSERT(newoff == newend);
    return 0;
}

static int lfs_dir_commit(lfs_t *lfs, lfs_dir_t *dir,
        const struct lfs_region *regions, int count) {
    // state for copying over
    const lfs_block_t oldpair[2] = {dir->pair[1], dir->pair[0]};
    bool relocated = false;
    int i, err;
    lfs_dir_t *d;

    // increment revision count
    dir->d.rev += 1;

    // keep pairs in order such that pair[0] is most recent
    lfs_pairswap(dir->pair);
    for (i = 0; i < count; i++) {
        dir->d.size += regions[i].newsize;
        dir->d.size -= regions[i].oldsize;
    }

    while (true) {
        if (true) {
            uint32_t crc = 0xffffffff;
            uint32_t ncrc = 0xffffffff;
            err = lfs_bd_erase(lfs, dir->pair[0]);
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    goto relocate;
                }
                return err;
            }

            // commit header
            lfs_dir_tole32(&dir->d);
            lfs_crc(&crc, &dir->d, sizeof(dir->d));
            err = lfs_bd_prog(lfs, dir->pair[0], 0, &dir->d, sizeof(dir->d));
            lfs_dir_fromle32(&dir->d);
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    goto relocate;
                }
                return err;
            }

            // commit region
            err = lfs_commit_region(lfs, &crc,
                    dir->pair[1], sizeof(dir->d),
                    dir->pair[0], sizeof(dir->d),
                    0, (0x7fffffff & dir->d.size)-sizeof(dir->d)-4,
                    regions, count);
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    goto relocate;
                }
                return err;
            }

            // commit crc
            crc = lfs_tole32(crc);
            err = lfs_bd_prog(lfs, dir->pair[0],
                    (0x7fffffff & dir->d.size)-4, &crc, 4);
            crc = lfs_fromle32(crc);
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    goto relocate;
                }
                return err;
            }

            err = lfs_bd_sync(lfs);
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    goto relocate;
                }
                return err;
            }

            // successful commit, check checksum to make sure
            err = lfs_bd_crc(lfs, dir->pair[0], 0,
                    (0x7fffffff & dir->d.size)-4, &ncrc);
            if (err) {
                return err;
            }

            if (ncrc != crc) {
                goto relocate;
            }
        }

        break;

relocate:
        //commit was corrupted
        LFS_DEBUG("Bad block at %d", dir->pair[0]);

        // drop caches and prepare to relocate block
        relocated = true;
        lfs->pcache.block = 0xffffffff;

        // can't relocate superblock, filesystem is now frozen
        if (lfs_paircmp(oldpair, (const lfs_block_t[2]){0, 1}) == 0) {
            LFS_WARN("Superblock %d has become unwritable", oldpair[0]);
            return LFS_ERR_CORRUPT;
        }

        // relocate half of pair
        err = lfs_alloc(lfs, &dir->pair[0]);
        if (err) {
            return err;
        }
    }

    if (relocated) {
        // update references if we relocated
        LFS_DEBUG("Relocating %d %d to %d %d",
                oldpair[0], oldpair[1], dir->pair[0], dir->pair[1]);
        err = lfs_relocate(lfs, oldpair, dir->pair);
        if (err) {
            return err;
        }
    }

    // shift over any directories that are affected
    for (d = lfs->dirs; d; d = d->next) {
        if (lfs_paircmp(d->pair, dir->pair) == 0) {
            d->pair[0] = dir->pair[0];
            d->pair[1] = dir->pair[1];
        }
    }

    return 0;
}

static int lfs_dir_get(lfs_t *lfs, const lfs_dir_t *dir,
        lfs_off_t off, void *buffer, lfs_size_t size) {
    return lfs_bd_read(lfs, dir->pair[0], off, buffer, size);
}

static int lfs_dir_set(lfs_t *lfs, lfs_dir_t *dir, lfs_entry_t *entry,
        struct lfs_region *regions, int count) {
    lfs_ssize_t diff = 0;
    lfs_size_t oldsize = entry->size;
    int i, err;
    lfs_file_t *f;
    lfs_dir_t *d;

    for (i = 0; i < count; i++) {
        diff += regions[i].newsize;
        diff -= regions[i].oldsize;
    }

    if (entry->off == 0) {
        entry->off = (0x7fffffff & dir->d.size) - 4;
    }

    if ((0x7fffffff & dir->d.size) + diff > lfs->cfg->block_size) {
        lfs_dir_t olddir = *dir;
        lfs_off_t oldoff = entry->off;
        lfs_dir_t pdir = olddir;

        if (oldsize) {
            // mark as moving
            uint8_t type;
            err = lfs_dir_get(lfs, &olddir, oldoff, &type, 1);
            if (err) {
                return err;
            }

            type |= LFS_STRUCT_MOVED;
            err = lfs_dir_commit(lfs, &olddir, (struct lfs_region[]){
                        {LFS_FROM_MEM, oldoff, 1, &type, 1}}, 1);
            if (err) {
                return err;
            }
        }

        // find available block or create a new one
        while ((0x7fffffff & dir->d.size) + oldsize + diff
                > lfs->cfg->block_size) {
            // we need to allocate a new dir block
            if (!(0x80000000 & dir->d.size)) {
                pdir = *dir;
                err = lfs_dir_alloc(lfs, dir);
                if (err) {
                    return err;
                }

                dir->d.tail[0] = pdir.d.tail[0];
                dir->d.tail[1] = pdir.d.tail[1];

                break;
            }

            err = lfs_dir_fetch(lfs, dir, dir->d.tail);
            if (err) {
                return err;
            }
        }

        // writing out new entry
        entry->off = dir->d.size - 4;
        entry->size += diff;
        err = lfs_dir_commit(lfs, dir, (struct lfs_region[]){
                {LFS_FROM_REGION, entry->off, 0, &(struct lfs_region_region){
                    olddir.pair[0], oldoff,
                    regions, count}, entry->size}}, 1);
        if (err) {
            return err;
        }

        // update pred dir, unless pred == old we can coalesce
        if (!oldsize || lfs_paircmp(pdir.pair, olddir.pair) != 0) {
            pdir.d.size |= 0x80000000;
            pdir.d.tail[0] = dir->pair[0];
            pdir.d.tail[1] = dir->pair[1];

            err = lfs_dir_commit(lfs, &pdir, NULL, 0);
            if (err) {
                return err;
            }
        } else if (oldsize) {
            olddir.d.size |= 0x80000000;
            olddir.d.tail[0] = dir->pair[0];
            olddir.d.tail[1] = dir->pair[1];
        }

        // remove old entry
        if (oldsize) {
            lfs_entry_t oldentry;
            oldentry.off = oldoff;
            err = lfs_dir_set(lfs, &olddir, &oldentry, (struct lfs_region[]){
                    {LFS_FROM_MEM, 0, oldsize, NULL, 0}}, 1);
            if (err) {
                return err;
            }
        }

        goto shift;
    }

    if ((0x7fffffff & dir->d.size) + diff == sizeof(dir->d)+4) {
        lfs_dir_t pdir;
        int res = lfs_pred(lfs, dir->pair, &pdir);
        if (res < 0) {
            return res;
        }

        if (pdir.d.size & 0x80000000) {
            pdir.d.size &= dir->d.size | 0x7fffffff;
            pdir.d.tail[0] = dir->d.tail[0];
            pdir.d.tail[1] = dir->d.tail[1];
            err = lfs_dir_commit(lfs, &pdir, NULL, 0);
            if (err) {
                return err;
            }
            goto shift;
        }
    }

    for (i = 0; i < count; i++) {
        regions[i].oldoff += entry->off;
    }

    err = lfs_dir_commit(lfs, dir, regions, count);
    if (err) {
        return err;
    }

    entry->size += diff;

shift:
    // shift over any files/directories that are affected
    for (f = lfs->files; f; f = f->next) {
        if (lfs_paircmp(f->pair, dir->pair) == 0) {
            if (f->pairoff == entry->off && entry->size == 0) {
                f->pair[0] = 0xffffffff;
                f->pair[1] = 0xffffffff;
            } else if (f->pairoff > entry->off) {
                f->pairoff += diff;
            }
        }
    }

    for (d = lfs->dirs; d; d = d->next) {
        if (lfs_paircmp(d->pair, dir->pair) == 0) {
            if (d->off > entry->off) {
                d->off += diff;
                d->pos += diff;
            }
        }
    }

    return 0;
}

static int lfs_dir_next(lfs_t *lfs, lfs_dir_t *dir, lfs_entry_t *entry) {
    int err;
    while (dir->off >= (0x7fffffff & dir->d.size)-4) {
        if (!(0x80000000 & dir->d.size)) {
            entry->off = dir->off;
            return LFS_ERR_NOENT;
        }

        err = lfs_dir_fetch(lfs, dir, dir->d.tail);
        if (err) {
            return err;
        }

        dir->off = sizeof(dir->d);
        dir->pos += sizeof(dir->d) + 4;
    }

    err = lfs_dir_get(lfs, dir, dir->off, &entry->d, sizeof(entry->d));
    lfs_entry_fromle32(&entry->d);
    if (err) {
        return err;
    }

    entry->off = dir->off;
    entry->size = lfs_entry_size(entry);
    dir->off += entry->size;
    dir->pos += entry->size;
    return 0;
}

static int lfs_dir_find(lfs_t *lfs, lfs_dir_t *dir,
        lfs_entry_t *entry, const char **path) {
    const char *pathname = *path;
    lfs_size_t pathlen;
    const char *suffix;
    lfs_size_t sufflen;
    int depth = 1;
    int err, res;

    while (true) {
    nextname:
        // skip slashes
        pathname += strspn(pathname, "/");
        pathlen = strcspn(pathname, "/");

        // special case for root dir
        if (pathname[0] == '\0') {
            *entry = (lfs_entry_t){
                .d.type = LFS_STRUCT_DIR | LFS_TYPE_DIR,
                .d.u.dir[0] = lfs->root[0],
                .d.u.dir[1] = lfs->root[1],
            };
            return 0;
        }

        // skip '.' and root '..'
        if ((pathlen == 1 && memcmp(pathname, ".", 1) == 0) ||
            (pathlen == 2 && memcmp(pathname, "..", 2) == 0)) {
            pathname += pathlen;
            goto nextname;
        }

        // skip if matched by '..' in name
        suffix = pathname + pathlen;
        sufflen = 0;
        depth = 1;
        while (true) {
            suffix += strspn(suffix, "/");
            sufflen = strcspn(suffix, "/");
            if (sufflen == 0) {
                break;
            }

            if (sufflen == 2 && memcmp(suffix, "..", 2) == 0) {
                depth -= 1;
                if (depth == 0) {
                    pathname = suffix + sufflen;
                    goto nextname;
                }
            } else {
                depth += 1;
            }

            suffix += sufflen;
        }

        // update what we've found
        *path = pathname;

        // find path
        while (true) {
            err = lfs_dir_next(lfs, dir, entry);
            if (err) {
                return err;
            }

            if (((0xf & entry->d.type) != LFS_TYPE_REG &&
                 (0xf & entry->d.type) != LFS_TYPE_DIR) ||
                entry->d.nlen != pathlen) {
                continue;
            }

            res = lfs_bd_cmp(lfs, dir->pair[0],
                    entry->off + entry->size - pathlen,
                    pathname, pathlen);
            if (res < 0) {
                return res;
            }

            // found match
            if (res) {
                break;
            }
        }

        // check that entry has not been moved
        if (entry->d.type & LFS_STRUCT_MOVED) {
            int moved = lfs_moved(lfs, &entry->d.u);
            if (moved < 0 || moved) {
                return (moved < 0) ? moved : LFS_ERR_NOENT;
            }

            entry->d.type &= ~LFS_STRUCT_MOVED;
        }

        pathname += pathlen;
        pathname += strspn(pathname, "/");
        if (pathname[0] == '\0') {
            return 0;
        }

        // continue on if we hit a directory
        if ((0xf & entry->d.type) != LFS_TYPE_DIR) {
            return LFS_ERR_NOTDIR;
        }

        err = lfs_dir_fetch(lfs, dir, entry->d.u.dir);
        if (err) {
            return err;
        }
    }
}

/// Internal attribute operations ///
static int lfs_dir_getinfo(lfs_t *lfs,
        lfs_dir_t *dir, const lfs_entry_t *entry, struct lfs_info *info) {
    memset(info, 0, sizeof(*info));
    info->type = 0xf & entry->d.type;
    if (entry->d.type == (LFS_STRUCT_CTZ | LFS_TYPE_REG)) {
        info->size = entry->d.u.file.size;
    } else if (entry->d.type == (LFS_STRUCT_INLINE | LFS_TYPE_REG)) {
        info->size = lfs_entry_elen(entry);
    }

    if ((0x70 & entry->d.type) == LFS_STRUCT_DIR &&
        lfs_paircmp(entry->d.u.dir, lfs->root) == 0) {
        strcpy(info->name, "/");
    } else {
        int err = lfs_dir_get(lfs, dir,
                entry->off + entry->size - entry->d.nlen,
                info->name, entry->d.nlen);
        if (err) {
            return err;
        }
    }

    return 0;
}

int lfs_dir_getattrs(lfs_t *lfs,
        lfs_dir_t *dir, const lfs_entry_t *entry,
        const struct lfs_attr *attrs, int count) {
    int j;
    lfs_off_t off, end;

    // set to zero in case we can't find the attributes or size mismatch
    for (j = 0; j < count; j++) {
        memset(attrs[j].buffer, 0, attrs[j].size);
    }

    // search for attribute in attribute region
    off = entry->off + 4+lfs_entry_elen(entry);
    end = off + lfs_entry_alen(entry);
    while (off < end) {
        lfs_entry_attr_t attr;
        int err = lfs_dir_get(lfs, dir, off, &attr.d, sizeof(attr.d));
        if (err) {
            return err;
        }

        for (j = 0; j < count; j++) {
            if (attrs[j].type == attr.d.type) {
                if (attrs[j].size < attr.d.len) {
                    return LFS_ERR_RANGE;
                }

                err = lfs_dir_get(lfs, dir, off+sizeof(attr.d),
                        attrs[j].buffer, attr.d.len);
                if (err) {
                    return err;
                }
            }
        }

        off += 2+attr.d.len;
    }

    return 0;
}

static lfs_ssize_t lfs_dir_checkattrs(lfs_t *lfs,
        lfs_dir_t *dir, lfs_entry_t *entry,
        const struct lfs_attr *attrs, int count) {
    int j;
    lfs_off_t off, end;
    // check that attributes fit
    // two separate passes so disk access is O(n)
    lfs_size_t nsize = 0;
    for (j = 0; j < count; j++) {
        if (attrs[j].size > 0) {
            nsize += 2+attrs[j].size;
        }
    }

    off = entry->off + 4+lfs_entry_elen(entry);
    end = off + lfs_entry_alen(entry);
    while (off < end) {
        lfs_entry_attr_t attr;
        bool updated = false;
        int err = lfs_dir_get(lfs, dir, off, &attr.d, sizeof(attr.d));
        if (err) {
            return err;
        }

        for (j = 0; j < count; j++) {
            if (attr.d.type == attrs[j].type) {
                updated = true;
            }
        }

        if (!updated) {
            nsize += 2+attr.d.len;
        }

        off += 2+attr.d.len;
    }

    if (nsize > lfs->attrs_size || (
            lfs_entry_size(entry) - lfs_entry_alen(entry) + nsize
                > lfs->cfg->block_size)) {
        return LFS_ERR_NOSPC;
    }

    return nsize;
}

static int lfs_dir_setattrs(lfs_t *lfs,
        lfs_dir_t *dir, lfs_entry_t *entry,
        const struct lfs_attr *attrs, int count) {
    // make sure attributes fit
    lfs_size_t oldlen = lfs_entry_alen(entry);
    lfs_ssize_t newlen = lfs_dir_checkattrs(lfs, dir, entry, attrs, count);
    if (newlen < 0) {
        return newlen;
    }

    // commit to entry, majority of work is in LFS_FROM_ATTRS
    entry->d.alen = (0xc0 & entry->d.alen) | newlen;
    return lfs_dir_set(lfs, dir, entry, (struct lfs_region[]){
            {LFS_FROM_MEM, 0, 4, &entry->d, 4},
            {LFS_FROM_ATTRS, 4+lfs_entry_elen(entry), oldlen,
                &(struct lfs_region_attrs){attrs, count}, newlen}}, 2);
}


/// Top level directory operations ///
int lfs_mkdir(lfs_t *lfs, const char *path) {
    lfs_dir_t cwd;
    lfs_entry_t entry;
    lfs_size_t nlen;
    lfs_dir_t dir;
    int err;

    // deorphan if we haven't yet, needed at most once after poweron
    if (!lfs->deorphaned) {
        err = lfs_deorphan(lfs);
        if (err) {
            return err;
        }
    }

    // fetch parent directory
    err = lfs_dir_fetch(lfs, &cwd, lfs->root);
    if (err) {
        return err;
    }

    err = lfs_dir_find(lfs, &cwd, &entry, &path);
    if (err != LFS_ERR_NOENT || strchr(path, '/') != NULL) {
        return err ? err : LFS_ERR_EXIST;
    }

    // check that name fits
    nlen = strlen(path);
    if (nlen > lfs->name_size) {
        return LFS_ERR_NAMETOOLONG;
    }

    // build up new directory
    lfs_alloc_ack(lfs);

    err = lfs_dir_alloc(lfs, &dir);
    if (err) {
        return err;
    }
    dir.d.tail[0] = cwd.d.tail[0];
    dir.d.tail[1] = cwd.d.tail[1];

    err = lfs_dir_commit(lfs, &dir, NULL, 0);
    if (err) {
        return err;
    }

    entry.d.type = LFS_STRUCT_DIR | LFS_TYPE_DIR;
    entry.d.elen = sizeof(entry.d) - 4;
    entry.d.alen = 0;
    entry.d.nlen = nlen;
    entry.d.u.dir[0] = dir.pair[0];
    entry.d.u.dir[1] = dir.pair[1];
    entry.size = 0;

    cwd.d.tail[0] = dir.pair[0];
    cwd.d.tail[1] = dir.pair[1];
    lfs_entry_tole32(&entry.d);
    err = lfs_dir_set(lfs, &cwd, &entry, (struct lfs_region[]){
            {LFS_FROM_MEM, 0, 0, &entry.d, sizeof(entry.d)},
            {LFS_FROM_MEM, 0, 0, path, nlen}}, 2);
    if (err) {
        return err;
    }

    lfs_alloc_ack(lfs);
    return 0;
}

int lfs_dir_open(lfs_t *lfs, lfs_dir_t *dir, const char *path) {
    lfs_entry_t entry;
    int err;

    dir->pair[0] = lfs->root[0];
    dir->pair[1] = lfs->root[1];

    err = lfs_dir_fetch(lfs, dir, dir->pair);
    if (err) {
        return err;
    }

    err = lfs_dir_find(lfs, dir, &entry, &path);
    if (err) {
        return err;
    } else if (entry.d.type != (LFS_STRUCT_DIR | LFS_TYPE_DIR)) {
        return LFS_ERR_NOTDIR;
    }

    err = lfs_dir_fetch(lfs, dir, entry.d.u.dir);
    if (err) {
        return err;
    }

    // setup head dir
    // special offset for '.' and '..'
    dir->head[0] = dir->pair[0];
    dir->head[1] = dir->pair[1];
    dir->pos = sizeof(dir->d) - 2;
    dir->off = sizeof(dir->d);

    // add to list of directories
    dir->next = lfs->dirs;
    lfs->dirs = dir;

    return 0;
}

int lfs_dir_close(lfs_t *lfs, lfs_dir_t *dir) {
    lfs_dir_t **p;
    // remove from list of directories
    for (p = &lfs->dirs; *p; p = &(*p)->next) {
        if (*p == dir) {
            *p = dir->next;
            break;
        }
    }

    return 0;
}

int lfs_dir_read(lfs_t *lfs, lfs_dir_t *dir, struct lfs_info *info) {
    lfs_entry_t entry;
    int err;

    memset(info, 0, sizeof(*info));

    // special offset for '.' and '..'
    if (dir->pos == sizeof(dir->d) - 2) {
        info->type = LFS_TYPE_DIR;
        strcpy(info->name, ".");
        dir->pos += 1;
        return 1;
    } else if (dir->pos == sizeof(dir->d) - 1) {
        info->type = LFS_TYPE_DIR;
        strcpy(info->name, "..");
        dir->pos += 1;
        return 1;
    }

    while (true) {
        err = lfs_dir_next(lfs, dir, &entry);
        if (err) {
            return (err == LFS_ERR_NOENT) ? 0 : err;
        }

        if ((0xf & entry.d.type) != LFS_TYPE_REG &&
            (0xf & entry.d.type) != LFS_TYPE_DIR) {
            continue;
        }

        // check that entry has not been moved
        if (entry.d.type & LFS_STRUCT_MOVED) {
            int moved = lfs_moved(lfs, &entry.d.u);
            if (moved < 0) {
                return moved;
            }

            if (moved) {
                continue;
            }

            entry.d.type &= ~LFS_STRUCT_MOVED;
        }

        break;
    }

    err = lfs_dir_getinfo(lfs, dir, &entry, info);
    if (err) {
        return err;
    }

    return 1;
}

int lfs_dir_seek(lfs_t *lfs, lfs_dir_t *dir, lfs_off_t off) {
    // simply walk from head dir
    int err = lfs_dir_rewind(lfs, dir);
    if (err) {
        return err;
    }
    dir->pos = off;

    while (off > (0x7fffffff & dir->d.size)) {
        off -= 0x7fffffff & dir->d.size;
        if (!(0x80000000 & dir->d.size)) {
            return LFS_ERR_INVAL;
        }

        err = lfs_dir_fetch(lfs, dir, dir->d.tail);
        if (err) {
            return err;
        }
    }

    dir->off = off;
    return 0;
}

lfs_soff_t lfs_dir_tell(lfs_t *lfs, lfs_dir_t *dir) {
    (void)lfs;
    return dir->pos;
}

int lfs_dir_rewind(lfs_t *lfs, lfs_dir_t *dir) {
    // reload the head dir
    int err = lfs_dir_fetch(lfs, dir, dir->head);
    if (err) {
        return err;
    }

    dir->pair[0] = dir->head[0];
    dir->pair[1] = dir->head[1];
    dir->pos = sizeof(dir->d) - 2;
    dir->off = sizeof(dir->d);
    return 0;
}


/// File index list operations ///
static int lfs_ctz_index(lfs_t *lfs, lfs_off_t *off) {
    lfs_off_t size = *off;
    lfs_off_t b = lfs->cfg->block_size - 2*4;
    lfs_off_t i = size / b;
    if (i == 0) {
        return 0;
    }

    i = (size - 4*(lfs_popc(i-1)+2)) / b;
    *off = size - b*i - 4*lfs_popc(i);
    return i;
}

static int lfs_ctz_find(lfs_t *lfs,
        lfs_cache_t *rcache, const lfs_cache_t *pcache,
        lfs_block_t head, lfs_size_t size,
        lfs_size_t pos, lfs_block_t *block, lfs_off_t *off) {
    lfs_off_t current_o, target;
    int err;
    if (size == 0) {
        *block = 0xffffffff;
        *off = 0;
        return 0;
    }

    current_o = lfs_ctz_index(lfs, &(lfs_off_t){size-1});
    target = lfs_ctz_index(lfs, &pos);

    while (current_o > target) {
        lfs_size_t skip = lfs_min(
                lfs_npw2(current_o-target+1) - 1,
                lfs_ctz(current_o));

        err = lfs_cache_read(lfs, rcache, pcache, head, 4*skip, &head, 4);
        head = lfs_fromle32(head);
        if (err) {
            return err;
        }

        LFS_ASSERT(head >= 2 && head <= lfs->cfg->block_count);
        current_o -= 1 << skip;
    }

    *block = head;
    *off = pos;
    return 0;
}

static int lfs_ctz_extend(lfs_t *lfs,
        lfs_cache_t *rcache, lfs_cache_t *pcache,
        lfs_block_t head, lfs_size_t size,
        lfs_block_t *block, lfs_off_t *off) {
    lfs_off_t i;
    lfs_off_t index;
    lfs_size_t skips;
    while (true) {
        // go ahead and grab a block
        lfs_block_t nblock;
        int err = lfs_alloc(lfs, &nblock);
        if (err) {
            return err;
        }
        LFS_ASSERT(nblock >= 2 && nblock <= lfs->cfg->block_count);

        if (true) {
            err = lfs_bd_erase(lfs, nblock);
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    goto relocate;
                }
                return err;
            }

            if (size == 0) {
                *block = nblock;
                *off = 0;
                return 0;
            }

            size -= 1;
            index = lfs_ctz_index(lfs, &size);
            size += 1;

            // just copy out the last block if it is incomplete
            if (size != lfs->cfg->block_size) {
                for (i = 0; i < size; i++) {
                    uint8_t data;
                    err = lfs_cache_read(lfs, rcache, NULL,
                            head, i, &data, 1);
                    if (err) {
                        return err;
                    }

                    err = lfs_cache_prog(lfs, pcache, rcache,
                            nblock, i, &data, 1);
                    if (err) {
                        if (err == LFS_ERR_CORRUPT) {
                            goto relocate;
                        }
                        return err;
                    }
                }

                *block = nblock;
                *off = size;
                return 0;
            }

            // append block
            index += 1;
            skips = lfs_ctz(index) + 1;

            for (i = 0; i < skips; i++) {
                head = lfs_tole32(head);
                err = lfs_cache_prog(lfs, pcache, rcache,
                        nblock, 4*i, &head, 4);
                head = lfs_fromle32(head);
                if (err) {
                    if (err == LFS_ERR_CORRUPT) {
                        goto relocate;
                    }
                    return err;
                }

                if (i != skips-1) {
                    err = lfs_cache_read(lfs, rcache, NULL,
                            head, 4*i, &head, 4);
                    head = lfs_fromle32(head);
                    if (err) {
                        return err;
                    }
                }

                LFS_ASSERT(head >= 2 && head <= lfs->cfg->block_count);
            }

            *block = nblock;
            *off = 4*skips;
            return 0;
        }

relocate:
        LFS_DEBUG("Bad block at %d", nblock);

        // just clear cache and try a new block
        pcache->block = 0xffffffff;
    }
}

static int lfs_ctz_traverse(lfs_t *lfs,
        lfs_cache_t *rcache, const lfs_cache_t *pcache,
        lfs_block_t head, lfs_size_t size,
        int (*cb)(void*, lfs_block_t), void *data) {
    int i;
    lfs_off_t index;
    if (size == 0) {
        return 0;
    }

    index = lfs_ctz_index(lfs, &(lfs_off_t){size-1});

    while (true) {
        int err = cb(data, head);
        lfs_block_t heads[2];
        int count;
        if (err) {
            return err;
        }

        if (index == 0) {
            return 0;
        }

        count = 2 - (index & 1);
        err = lfs_cache_read(lfs, rcache, pcache, head, 0, &heads, count*4);
        heads[0] = lfs_fromle32(heads[0]);
        heads[1] = lfs_fromle32(heads[1]);
        if (err) {
            return err;
        }

        for (i = 0; i < count-1; i++) {
            err = cb(data, heads[i]);
            if (err) {
                return err;
            }
        }

        head = heads[count-1];
        index -= count;
    }
}


/// Top level file operations ///
int lfs_file_open(lfs_t *lfs, lfs_file_t *file,
        const char *path, int flags) {
    lfs_dir_t cwd;
    lfs_entry_t entry;
    lfs_size_t nlen;
    int err;
    // deorphan if we haven't yet, needed at most once after poweron
    if ((flags & 3) != LFS_O_RDONLY && !lfs->deorphaned) {
        int err = lfs_deorphan(lfs);
        if (err) {
            return err;
        }
    }

    // allocate entry for file if it doesn't exist
    err = lfs_dir_fetch(lfs, &cwd, lfs->root);
    if (err) {
        return err;
    }

    err = lfs_dir_find(lfs, &cwd, &entry, &path);
    if (err && (err != LFS_ERR_NOENT || strchr(path, '/') != NULL)) {
        return err;
    }

    if (err == LFS_ERR_NOENT) {
        if (!(flags & LFS_O_CREAT)) {
            return LFS_ERR_NOENT;
        }

        // check that name fits
        nlen = strlen(path);
        if (nlen > lfs->name_size) {
            return LFS_ERR_NAMETOOLONG;
        }

        // create entry to remember name
        entry.d.type = LFS_STRUCT_INLINE | LFS_TYPE_REG;
        entry.d.elen = 0;
        entry.d.alen = 0;
        entry.d.nlen = nlen;
        entry.size = 0;

        err = lfs_dir_set(lfs, &cwd, &entry, (struct lfs_region[]){
                {LFS_FROM_MEM, 0, 0, &entry.d, 4},
                {LFS_FROM_MEM, 0, 0, path, nlen}}, 2);
        if (err) {
            return err;
        }
    } else if ((0xf & entry.d.type) == LFS_TYPE_DIR) {
        return LFS_ERR_ISDIR;
    } else if (flags & LFS_O_EXCL) {
        return LFS_ERR_EXIST;
    }

    // allocate buffer if needed
    file->cache.block = 0xffffffff;
    if (lfs->cfg->file_buffer) {
        file->cache.buffer = lfs->cfg->file_buffer;
    } else if ((file->flags & 3) == LFS_O_RDONLY) {
        file->cache.buffer = lfs_malloc(lfs->cfg->read_size);
        if (!file->cache.buffer) {
            return LFS_ERR_NOMEM;
        }
    } else {
        file->cache.buffer = lfs_malloc(lfs->cfg->prog_size);
        if (!file->cache.buffer) {
            return LFS_ERR_NOMEM;
        }
    }

    // setup file struct
    file->pair[0] = cwd.pair[0];
    file->pair[1] = cwd.pair[1];
    file->pairoff = entry.off;
    file->flags = flags;
    file->pos = 0;

    // calculate max inline size based on the size of the entry
    file->inline_size = lfs_min(lfs->inline_size,
        lfs->cfg->block_size - (sizeof(cwd.d)+4) -
        (lfs_entry_size(&entry) - lfs_entry_elen(&entry)));

    if ((0x70 & entry.d.type) == LFS_STRUCT_INLINE) {
        // load inline files
        file->head = 0xfffffffe;
        file->size = lfs_entry_elen(&entry);
        file->flags |= LFS_F_INLINE;
        file->cache.block = file->head;
        file->cache.off = 0;
        err = lfs_dir_get(lfs, &cwd,
                entry.off + 4,
                file->cache.buffer, file->size);
        if (err) {
            lfs_free(file->cache.buffer);
            return err;
        }
    } else {
        // use ctz list from entry
        file->head = entry.d.u.file.head;
        file->size = entry.d.u.file.size;
    }

    // truncate if requested
    if (flags & LFS_O_TRUNC) {
        if (file->size != 0) {
            file->flags |= LFS_F_DIRTY;
        }

        file->head = 0xfffffffe;
        file->size = 0;
        file->flags |= LFS_F_INLINE;
        file->cache.block = file->head;
        file->cache.off = 0;
    }

    // add to list of files
    file->next = lfs->files;
    lfs->files = file;

    return 0;
}

int lfs_file_close(lfs_t *lfs, lfs_file_t *file) {
    lfs_file_t **p;
    int err = lfs_file_sync(lfs, file);

    // remove from list of files
    for (p = &lfs->files; *p; p = &(*p)->next) {
        if (*p == file) {
            *p = file->next;
            break;
        }
    }

    // clean up memory
    if (!lfs->cfg->file_buffer) {
        lfs_free(file->cache.buffer);
    }

    return err;
}

static int lfs_file_relocate(lfs_t *lfs, lfs_file_t *file) {
    lfs_off_t i;
    lfs_block_t nblock;
    int err;
relocate:;
    // just relocate what exists into new block
    err = lfs_alloc(lfs, &nblock);
    if (err) {
        return err;
    }

    err = lfs_bd_erase(lfs, nblock);
    if (err) {
        if (err == LFS_ERR_CORRUPT) {
            goto relocate;
        }
        return err;
    }

    // either read from dirty cache or disk
    for (i = 0; i < file->off; i++) {
        uint8_t data;
        err = lfs_cache_read(lfs, &lfs->rcache, &file->cache,
                file->block, i, &data, 1);
        if (err) {
            return err;
        }

        err = lfs_cache_prog(lfs, &lfs->pcache, &lfs->rcache,
                nblock, i, &data, 1);
        if (err) {
            if (err == LFS_ERR_CORRUPT) {
                goto relocate;
            }
            return err;
        }
    }

    // copy over new state of file
    memcpy(file->cache.buffer, lfs->pcache.buffer, lfs->cfg->prog_size);
    file->cache.block = lfs->pcache.block;
    file->cache.off = lfs->pcache.off;
    lfs->pcache.block = 0xffffffff;

    file->block = nblock;
    return 0;
}

static int lfs_file_flush(lfs_t *lfs, lfs_file_t *file) {
    if (file->flags & LFS_F_READING) {
        file->flags &= ~LFS_F_READING;
    }

    if (file->flags & LFS_F_WRITING) {
        lfs_off_t pos = file->pos;

        if (!(file->flags & LFS_F_INLINE)) {
            // copy over anything after current branch
            lfs_file_t orig = {
                .head = file->head,
                .size = file->size,
                .flags = LFS_O_RDONLY,
                .pos = file->pos,
                .cache = lfs->rcache,
            };
            lfs->rcache.block = 0xffffffff;

            while (file->pos < file->size) {
                // copy over a byte at a time, leave it up to caching
                // to make this efficient
                uint8_t data;
                lfs_ssize_t res = lfs_file_read(lfs, &orig, &data, 1);
                if (res < 0) {
                    return res;
                }

                res = lfs_file_write(lfs, file, &data, 1);
                if (res < 0) {
                    return res;
                }

                // keep our reference to the rcache in sync
                if (lfs->rcache.block != 0xffffffff) {
                    orig.cache.block = 0xffffffff;
                    lfs->rcache.block = 0xffffffff;
                }
            }

            // write out what we have
            while (true) {
                int err = lfs_cache_flush(lfs, &file->cache, &lfs->rcache);
                if (err) {
                    if (err == LFS_ERR_CORRUPT) {
                        goto relocate;
                    }
                    return err;
                }

                break;
relocate:
                LFS_DEBUG("Bad block at %d", file->block);
                err = lfs_file_relocate(lfs, file);
                if (err) {
                    return err;
                }
            }
        } else {
            file->size = lfs_max(file->pos, file->size);
        }

        // actual file updates
        file->head = file->block;
        file->size = file->pos;
        file->flags &= ~LFS_F_WRITING;
        file->flags |= LFS_F_DIRTY;

        file->pos = pos;
    }

    return 0;
}

static int lfs_file_check_for_attr_changes(lfs_t *lfs, lfs_file_t *file) {
    int i = 0;

    if (file->flags & LFS_F_DIRTY) {
        // No point checking, we'll rewrite anyway
        return 0;
    }

    if (!lfs_pairisnull(file->pair)) {
        lfs_dir_t cwd;
        lfs_entry_t entry = {.off = file->pairoff};
        int err = lfs_dir_fetch(lfs, &cwd, file->pair);
        if (err) {
            return err;
        }

        err = lfs_dir_get(lfs, &cwd, entry.off, &entry.d, 4);
        if (err) {
            return err;
        }
        entry.size = lfs_entry_size(&entry);

        // For each attribute, check if they've changed
        for (i = 0; i < file->attrcount; i++) {
            struct lfs_attr attr;
            uint8_t buffer[file->attrs[i].size];

            attr.type = file->attrs[i].type;
            attr.size = file->attrs[i].size;
            attr.buffer = buffer;

            err = lfs_dir_getattrs(lfs, &cwd, &entry, &attr, 1);
            if (err) {
                return err;
            }

            if (memcmp(file->attrs[i].buffer, buffer, file->attrs[i].size) != 0) {
                file->flags |= LFS_F_DIRTY;
                return 0;
            }
        }
    }

    return 0;
}

int lfs_file_sync(lfs_t *lfs, lfs_file_t *file) {
    int err = lfs_file_flush(lfs, file);
    if (err) {
        return err;
    }

    err = lfs_file_check_for_attr_changes(lfs, file);
    if (err) {
        return err;
    }

    if ((file->flags & LFS_F_DIRTY) &&
            !(file->flags & LFS_F_ERRED) &&
            !lfs_pairisnull(file->pair)) {
        // update dir entry
        lfs_dir_t cwd;
        lfs_entry_t entry = {.off = file->pairoff};
        lfs_size_t oldelen;
        lfs_size_t oldalen;
        lfs_ssize_t newalen;
        const void *buffer;
        lfs_size_t size;

        err = lfs_dir_fetch(lfs, &cwd, file->pair);
        if (err) {
            return err;
        }

        err = lfs_dir_get(lfs, &cwd, entry.off, &entry.d, 4);
        if (err) {
            return err;
        }
        entry.size = lfs_entry_size(&entry);

        LFS_ASSERT((0xf & entry.d.type) == LFS_TYPE_REG);
        oldelen = lfs_entry_elen(&entry);
        oldalen = lfs_entry_alen(&entry);

        // either update the references or inline the whole file
        if (!(file->flags & LFS_F_INLINE)) {
            entry.d.type = LFS_STRUCT_CTZ | LFS_TYPE_REG;
            entry.d.u.file.head = file->head;
            entry.d.u.file.size = file->size;

            lfs_entry_tole32(&entry.d);
            buffer = (const uint8_t *)&entry.d + 4;
            size = sizeof(entry.d) - 4;
        } else {
            entry.d.type = LFS_STRUCT_INLINE | LFS_TYPE_REG;

            buffer = file->cache.buffer;
            size = file->size;
        }

        // get new alen from disk
        newalen = lfs_dir_checkattrs(lfs, &cwd, &entry,
                file->attrs, file->attrcount);
        if (newalen < 0) {
            return newalen;
        }

        entry.d.elen = size & 0xff;
        entry.d.alen = (newalen & 0x3f) | ((size >> 2) & 0xc0);

        // write out update
        err = lfs_dir_set(lfs, &cwd, &entry, (struct lfs_region[]){
                {LFS_FROM_MEM, 0, 4, &entry.d, 4},
                {LFS_FROM_MEM, 4, oldelen, buffer, size},
                {LFS_FROM_ATTRS, 4+oldelen, oldalen,
                    &(struct lfs_region_attrs){file->attrs, file->attrcount},
                    newalen}}, 3);
        if (err) {
            return err;
        }

        file->flags &= ~LFS_F_DIRTY;
    }

    return 0;
}

lfs_ssize_t lfs_file_read(lfs_t *lfs, lfs_file_t *file,
        void *buffer, lfs_size_t size) {
    uint8_t *data = buffer;
    lfs_size_t nsize = size;
    lfs_size_t diff;
    int err;

    if ((file->flags & 3) == LFS_O_WRONLY) {
        return LFS_ERR_BADF;
    }

    if (file->flags & LFS_F_WRITING) {
        // flush out any writes
        err = lfs_file_flush(lfs, file);
        if (err) {
            return err;
        }
    }

    if (file->pos >= file->size) {
        // eof if past end
        return 0;
    }

    size = lfs_min(size, file->size - file->pos);
    nsize = size;

    while (nsize > 0) {
        // check if we need a new block
        if (!(file->flags & LFS_F_READING) ||
                file->off == lfs->cfg->block_size) {
            if (!(file->flags & LFS_F_INLINE)) {
                err = lfs_ctz_find(lfs, &file->cache, NULL,
                        file->head, file->size,
                        file->pos, &file->block, &file->off);
                if (err) {
                    return err;
                }
            } else {
                file->block = 0xfffffffe;
                file->off = file->pos;
            }

            file->flags |= LFS_F_READING;
        }

        // read as much as we can in current block
        diff = lfs_min(nsize, lfs->cfg->block_size - file->off);
        err = lfs_cache_read(lfs, &file->cache, NULL,
                file->block, file->off, data, diff);
        if (err) {
            return err;
        }

        file->pos += diff;
        file->off += diff;
        data += diff;
        nsize -= diff;
    }

    return size;
}

lfs_ssize_t lfs_file_write(lfs_t *lfs, lfs_file_t *file,
        const void *buffer, lfs_size_t size) {
    const uint8_t *data = buffer;
    lfs_size_t nsize = size;
    lfs_size_t diff;
    int err;

    if ((file->flags & 3) == LFS_O_RDONLY) {
        return LFS_ERR_BADF;
    }

    if (file->flags & LFS_F_READING) {
        // drop any reads
        err = lfs_file_flush(lfs, file);
        if (err) {
            return err;
        }
    }

    if ((file->flags & LFS_O_APPEND) && file->pos < file->size) {
        file->pos = file->size;
    }

    if (!(file->flags & LFS_F_WRITING) && file->pos > file->size) {
        // fill with zeros
        lfs_off_t pos = file->pos;
        file->pos = file->size;

        while (file->pos < pos) {
            lfs_ssize_t res = lfs_file_write(lfs, file, &(uint8_t){0}, 1);
            if (res < 0) {
                return res;
            }
        }
    }

    if ((file->flags & LFS_F_INLINE) &&
            file->pos + nsize >= file->inline_size) {
        // inline file doesn't fit anymore
        file->block = 0xfffffffe;
        file->off = file->pos;

        lfs_alloc_ack(lfs);
        err = lfs_file_relocate(lfs, file);
        if (err) {
            file->flags |= LFS_F_ERRED;
            return err;
        }

        file->flags &= ~LFS_F_INLINE;
        file->flags |= LFS_F_WRITING;
    }

    while (nsize > 0) {
        // check if we need a new block
        if (!(file->flags & LFS_F_WRITING) ||
                file->off == lfs->cfg->block_size) {
            if (!(file->flags & LFS_F_INLINE)) {
                if (!(file->flags & LFS_F_WRITING) && file->pos > 0) {
                    // find out which block we're extending from
                    err = lfs_ctz_find(lfs, &file->cache, NULL,
                            file->head, file->size,
                            file->pos-1, &file->block, &file->off);
                    if (err) {
                        file->flags |= LFS_F_ERRED;
                        return err;
                    }

                    // mark cache as dirty since we may have read data into it
                    file->cache.block = 0xffffffff;
                }

                // extend file with new blocks
                lfs_alloc_ack(lfs);
                err = lfs_ctz_extend(lfs, &lfs->rcache, &file->cache,
                        file->block, file->pos,
                        &file->block, &file->off);
                if (err) {
                    file->flags |= LFS_F_ERRED;
                    return err;
                }
            } else {
                file->block = 0xfffffffe;
                file->off = file->pos;
            }

            file->flags |= LFS_F_WRITING;
        }

        // program as much as we can in current block
        diff = lfs_min(nsize, lfs->cfg->block_size - file->off);
        while (true) {
            err = lfs_cache_prog(lfs, &file->cache, &lfs->rcache,
                    file->block, file->off, data, diff);
            if (err) {
                if (err == LFS_ERR_CORRUPT) {
                    goto relocate;
                }
                file->flags |= LFS_F_ERRED;
                return err;
            }

            break;
relocate:
            err = lfs_file_relocate(lfs, file);
            if (err) {
                file->flags |= LFS_F_ERRED;
                return err;
            }
        }

        file->pos += diff;
        file->off += diff;
        data += diff;
        nsize -= diff;

        lfs_alloc_ack(lfs);
    }

    file->flags &= ~LFS_F_ERRED;
    return size;
}

lfs_soff_t lfs_file_seek(lfs_t *lfs, lfs_file_t *file,
        lfs_soff_t off, int whence) {
    // write out everything beforehand, may be noop if rdonly
    int err = lfs_file_flush(lfs, file);
    if (err) {
        return err;
    }

    // update pos
    if (whence == LFS_SEEK_SET) {
        file->pos = off;
    } else if (whence == LFS_SEEK_CUR) {
        if (off < 0 && (lfs_off_t)-off > file->pos) {
            return LFS_ERR_INVAL;
        }

        file->pos = file->pos + off;
    } else if (whence == LFS_SEEK_END) {
        if (off < 0 && (lfs_off_t)-off > file->size) {
            return LFS_ERR_INVAL;
        }

        file->pos = file->size + off;
    }

    return file->pos;
}

int lfs_file_truncate(lfs_t *lfs, lfs_file_t *file, lfs_off_t size) {
    lfs_off_t oldsize;
    int err;

    if ((file->flags & 3) == LFS_O_RDONLY) {
        return LFS_ERR_BADF;
    }

    oldsize = lfs_file_size(lfs, file);
    if (size < oldsize) {
        // need to flush since directly changing metadata
        err = lfs_file_flush(lfs, file);
        if (err) {
            return err;
        }

        // lookup new head in ctz skip list
        err = lfs_ctz_find(lfs, &file->cache, NULL,
                file->head, file->size,
                size, &file->head, &(lfs_off_t){0});
        if (err) {
            return err;
        }

        file->size = size;
        file->flags |= LFS_F_DIRTY;
    } else if (size > oldsize) {
        lfs_off_t pos = file->pos;

        // flush+seek if not already at end
        if (file->pos != oldsize) {
            err = lfs_file_seek(lfs, file, 0, LFS_SEEK_END);
            if (err < 0) {
                return err;
            }
        }

        // fill with zeros
        while (file->pos < size) {
            lfs_ssize_t res = lfs_file_write(lfs, file, &(uint8_t){0}, 1);
            if (res < 0) {
                return res;
            }
        }

        // restore pos
        err = lfs_file_seek(lfs, file, pos, LFS_SEEK_SET);
        if (err < 0) {
            return err;
        }
    }

    return 0;
}

lfs_soff_t lfs_file_tell(lfs_t *lfs, lfs_file_t *file) {
    (void)lfs;
    return file->pos;
}

int lfs_file_rewind(lfs_t *lfs, lfs_file_t *file) {
    lfs_soff_t res = lfs_file_seek(lfs, file, 0, LFS_SEEK_SET);
    if (res < 0) {
        return res;
    }

    return 0;
}

lfs_soff_t lfs_file_size(lfs_t *lfs, lfs_file_t *file) {
    (void)lfs;
    if (file->flags & LFS_F_WRITING) {
        return lfs_max(file->pos, file->size);
    } else {
        return file->size;
    }
}

int lfs_file_getattrs(lfs_t *lfs, lfs_file_t *file,
        const struct lfs_attr *attrs, int count) {
    int i;
    int j;
    // set to null in case we can't find the attrs (missing file?)
    for (j = 0; j < count; j++) {
        memset(attrs[j].buffer, 0, attrs[j].size);
    }

    // load from disk if we haven't already been deleted
    if (!lfs_pairisnull(file->pair)) {
        lfs_dir_t cwd;
        lfs_entry_t entry = {.off = file->pairoff};
        int err = lfs_dir_fetch(lfs, &cwd, file->pair);
        if (err) {
            return err;
        }

        err = lfs_dir_get(lfs, &cwd, entry.off, &entry.d, 4);
        if (err) {
            return err;
        }
        entry.size = lfs_entry_size(&entry);

        err = lfs_dir_getattrs(lfs, &cwd, &entry, attrs, count);
        if (err) {
            return err;
        }
    }

    // override an attrs we have stored locally
    for (i = 0; i < file->attrcount; i++) {
        for (j = 0; j < count; j++) {
            if (attrs[j].type == file->attrs[i].type) {
                if (attrs[j].size < file->attrs[i].size) {
                    return LFS_ERR_RANGE;
                }

                memset(attrs[j].buffer, 0, attrs[j].size);
                memcpy(attrs[j].buffer,
                        file->attrs[i].buffer, file->attrs[i].size);
            }
        }
    }

    return 0;
}

int lfs_file_setattrs(lfs_t *lfs, lfs_file_t *file,
        const struct lfs_attr *attrs, int count) {
    if ((file->flags & 3) == LFS_O_RDONLY) {
        return LFS_ERR_BADF;
    }

    // at least make sure attributes fit
    if (!lfs_pairisnull(file->pair)) {
        lfs_dir_t cwd;
        lfs_ssize_t res;
        lfs_entry_t entry = {.off = file->pairoff};
        int err = lfs_dir_fetch(lfs, &cwd, file->pair);
        if (err) {
            return err;
        }

        err = lfs_dir_get(lfs, &cwd, entry.off, &entry.d, 4);
        if (err) {
            return err;
        }
        entry.size = lfs_entry_size(&entry);

        res = lfs_dir_checkattrs(lfs, &cwd, &entry, attrs, count);
        if (res < 0) {
            return res;
        }
    }

    // just tack to the file, will be written at sync time
    file->attrs = attrs;
    file->attrcount = count;

    return 0;
}


/// General fs operations ///
int lfs_stat(lfs_t *lfs, const char *path, struct lfs_info *info) {
    lfs_dir_t cwd;
    lfs_entry_t entry;
    int err = lfs_dir_fetch(lfs, &cwd, lfs->root);
    if (err) {
        return err;
    }

    err = lfs_dir_find(lfs, &cwd, &entry, &path);
    if (err) {
        return err;
    }

    return lfs_dir_getinfo(lfs, &cwd, &entry, info);
}

int lfs_remove(lfs_t *lfs, const char *path) {
    lfs_dir_t cwd;
    lfs_entry_t entry;
    lfs_dir_t dir;
    int err;
    // deorphan if we haven't yet, needed at most once after poweron
    if (!lfs->deorphaned) {
        err = lfs_deorphan(lfs);
        if (err) {
            return err;
        }
    }

    err = lfs_dir_fetch(lfs, &cwd, lfs->root);
    if (err) {
        return err;
    }

    err = lfs_dir_find(lfs, &cwd, &entry, &path);
    if (err) {
        return err;
    }

    if ((0xf & entry.d.type) == LFS_TYPE_DIR) {
        // must be empty before removal, checking size
        // without masking top bit checks for any case where
        // dir is not empty
        err = lfs_dir_fetch(lfs, &dir, entry.d.u.dir);
        if (err) {
            return err;
        } else if (dir.d.size != sizeof(dir.d)+4) {
            return LFS_ERR_NOTEMPTY;
        }
    }

    // remove the entry
    err = lfs_dir_set(lfs, &cwd, &entry, (struct lfs_region[]){
            {LFS_FROM_MEM, 0, entry.size, NULL, 0}}, 1);
    if (err) {
        return err;
    }

    // if we were a directory, find pred, replace tail
    if ((0xf & entry.d.type) == LFS_TYPE_DIR) {
        int res = lfs_pred(lfs, dir.pair, &cwd);
        if (res < 0) {
            return res;
        }

        LFS_ASSERT(res); // must have pred
        cwd.d.tail[0] = dir.d.tail[0];
        cwd.d.tail[1] = dir.d.tail[1];

        err = lfs_dir_commit(lfs, &cwd, NULL, 0);
        if (err) {
            return err;
        }
    }

    return 0;
}

int lfs_rename(lfs_t *lfs, const char *oldpath, const char *newpath) {
    lfs_entry_t oldentry;
    lfs_dir_t oldcwd;
    lfs_dir_t newcwd;
    lfs_dir_t dir;
    lfs_entry_t preventry;
    lfs_entry_t newentry;
    lfs_size_t newsize;
    lfs_size_t nlen;
    bool prevexists, samepair;
    int err;
    // deorphan if we haven't yet, needed at most once after poweron
    if (!lfs->deorphaned) {
        err = lfs_deorphan(lfs);
        if (err) {
            return err;
        }
    }

    // find old entry
    err = lfs_dir_fetch(lfs, &oldcwd, lfs->root);
    if (err) {
        return err;
    }

    err = lfs_dir_find(lfs, &oldcwd, &oldentry, &oldpath);
    if (err) {
        return err;
    }

    // allocate new entry
    err = lfs_dir_fetch(lfs, &newcwd, lfs->root);
    if (err) {
        return err;
    }

    err = lfs_dir_find(lfs, &newcwd, &preventry, &newpath);
    if (err && (err != LFS_ERR_NOENT || strchr(newpath, '/') != NULL)) {
        return err;
    }

    prevexists = (err != LFS_ERR_NOENT);
    samepair = (lfs_paircmp(oldcwd.pair, newcwd.pair) == 0);

    // check that name fits
    nlen = strlen(newpath);
    if (nlen > lfs->name_size) {
        return LFS_ERR_NAMETOOLONG;
    }

    if (oldentry.size - oldentry.d.nlen + nlen > lfs->cfg->block_size) {
        return LFS_ERR_NOSPC;
    }

    // must have same type
    if (prevexists && preventry.d.type != oldentry.d.type) {
        return LFS_ERR_ISDIR;
    }

    if (prevexists && (0xf & preventry.d.type) == LFS_TYPE_DIR) {
        // must be empty before removal, checking size
        // without masking top bit checks for any case where
        // dir is not empty
        err = lfs_dir_fetch(lfs, &dir, preventry.d.u.dir);
        if (err) {
            return err;
        } else if (dir.d.size != sizeof(dir.d)+4) {
            return LFS_ERR_NOTEMPTY;
        }
    }

    // mark as moving
    oldentry.d.type |= LFS_STRUCT_MOVED;
    err = lfs_dir_set(lfs, &oldcwd, &oldentry, (struct lfs_region[]){
            {LFS_FROM_MEM, 0, 1, &oldentry.d.type, 1}}, 1);
    oldentry.d.type &= ~LFS_STRUCT_MOVED;
    if (err) {
        return err;
    }

    // update pair if newcwd == oldcwd
    if (samepair) {
        newcwd = oldcwd;
    }

    // move to new location
    newentry = preventry;
    newentry.d = oldentry.d;
    newentry.d.type &= ~LFS_STRUCT_MOVED;
    newentry.d.nlen = nlen;
    newentry.size = prevexists ? preventry.size : 0;

    newsize = oldentry.size - oldentry.d.nlen + newentry.d.nlen;
    err = lfs_dir_set(lfs, &newcwd, &newentry, (struct lfs_region[]){
            {LFS_FROM_REGION, 0, prevexists ? preventry.size : 0,
                &(struct lfs_region_region){
                    oldcwd.pair[0], oldentry.off, (struct lfs_region[]){
                        {LFS_FROM_MEM, 0, 4, &newentry.d, 4},
                        {LFS_FROM_MEM, newsize-nlen, 0, newpath, nlen}}, 2},
                newsize}}, 1);
    if (err) {
        return err;
    }

    // update pair if newcwd == oldcwd
    if (samepair) {
        oldcwd = newcwd;
    }

    // remove old entry
    err = lfs_dir_set(lfs, &oldcwd, &oldentry, (struct lfs_region[]){
            {LFS_FROM_MEM, 0, oldentry.size, NULL, 0}}, 1);
    if (err) {
        return err;
    }

    // if we were a directory, find pred, replace tail
    if (prevexists && (0xf & preventry.d.type) == LFS_TYPE_DIR) {
        int res = lfs_pred(lfs, dir.pair, &newcwd);
        if (res < 0) {
            return res;
        }

        LFS_ASSERT(res); // must have pred
        newcwd.d.tail[0] = dir.d.tail[0];
        newcwd.d.tail[1] = dir.d.tail[1];

        err = lfs_dir_commit(lfs, &newcwd, NULL, 0);
        if (err) {
            return err;
        }
    }

    return 0;
}

int lfs_getattrs(lfs_t *lfs, const char *path,
        const struct lfs_attr *attrs, int count) {
    lfs_dir_t cwd;
    lfs_entry_t entry;
    int err = lfs_dir_fetch(lfs, &cwd, lfs->root);
    if (err) {
        return err;
    }

    err = lfs_dir_find(lfs, &cwd, &entry, &path);
    if (err) {
        return err;
    }

    return lfs_dir_getattrs(lfs, &cwd, &entry, attrs, count);
}

int lfs_setattrs(lfs_t *lfs, const char *path,
        const struct lfs_attr *attrs, int count) {
    lfs_dir_t cwd;
    lfs_entry_t entry;
    int err = lfs_dir_fetch(lfs, &cwd, lfs->root);
    if (err) {
        return err;
    }

    err = lfs_dir_find(lfs, &cwd, &entry, &path);
    if (err) {
        return err;
    }

    return lfs_dir_setattrs(lfs, &cwd, &entry, attrs, count);
}


/// Filesystem operations ///
static int lfs_init(lfs_t *lfs, const struct lfs_config *cfg) {
    lfs->cfg = cfg;

    // setup read cache
    lfs->rcache.block = 0xffffffff;
    if (lfs->cfg->read_buffer) {
        lfs->rcache.buffer = lfs->cfg->read_buffer;
    } else {
        lfs->rcache.buffer = lfs_malloc(lfs->cfg->read_size);
        if (!lfs->rcache.buffer) {
            return LFS_ERR_NOMEM;
        }
    }

    // setup program cache
    lfs->pcache.block = 0xffffffff;
    if (lfs->cfg->prog_buffer) {
        lfs->pcache.buffer = lfs->cfg->prog_buffer;
    } else {
        lfs->pcache.buffer = lfs_malloc(lfs->cfg->prog_size);
        if (!lfs->pcache.buffer) {
            return LFS_ERR_NOMEM;
        }
    }

    // setup lookahead, round down to nearest 32-bits
    LFS_ASSERT(lfs->cfg->lookahead % 32 == 0);
    LFS_ASSERT(lfs->cfg->lookahead > 0);
    if (lfs->cfg->lookahead_buffer) {
        lfs->free.buffer = lfs->cfg->lookahead_buffer;
    } else {
        lfs->free.buffer = lfs_malloc(lfs->cfg->lookahead/8);
        if (!lfs->free.buffer) {
            return LFS_ERR_NOMEM;
        }
    }

    // check that program and read sizes are multiples of the block size
    LFS_ASSERT(lfs->cfg->prog_size % lfs->cfg->read_size == 0);
    LFS_ASSERT(lfs->cfg->block_size % lfs->cfg->prog_size == 0);

    // check that the block size is large enough to fit ctz pointers
    LFS_ASSERT(4*lfs_npw2(0xffffffff / (lfs->cfg->block_size-2*4))
            <= lfs->cfg->block_size);

    // check that the size limits are sane
    LFS_ASSERT(lfs->cfg->inline_size <= LFS_INLINE_MAX);
    LFS_ASSERT(lfs->cfg->inline_size <= lfs->cfg->read_size);
    lfs->inline_size = lfs->cfg->inline_size;
    if (!lfs->inline_size) {
        lfs->inline_size = lfs_min(LFS_INLINE_MAX, lfs->cfg->read_size);
    }

    LFS_ASSERT(lfs->cfg->attrs_size <= LFS_ATTRS_MAX);
    lfs->attrs_size = lfs->cfg->attrs_size;
    if (!lfs->attrs_size) {
        lfs->attrs_size = LFS_ATTRS_MAX;
    }

    LFS_ASSERT(lfs->cfg->name_size <= LFS_NAME_MAX);
    lfs->name_size = lfs->cfg->name_size;
    if (!lfs->name_size) {
        lfs->name_size = LFS_NAME_MAX;
    }

    // setup default state
    lfs->root[0] = 0xffffffff;
    lfs->root[1] = 0xffffffff;
    lfs->files = NULL;
    lfs->dirs = NULL;
    lfs->deorphaned = false;

    return 0;
}

static int lfs_deinit(lfs_t *lfs) {
    // free allocated memory
    if (!lfs->cfg->read_buffer) {
        lfs_free(lfs->rcache.buffer);
    }

    if (!lfs->cfg->prog_buffer) {
        lfs_free(lfs->pcache.buffer);
    }

    if (!lfs->cfg->lookahead_buffer) {
        lfs_free(lfs->free.buffer);
    }

    return 0;
}

int lfs_format(lfs_t *lfs, const struct lfs_config *cfg) {
    lfs_dir_t root;
    lfs_dir_t superdir;
    lfs_superblock_t superblock;
    lfs_entry_t superentry;
    int err = lfs_init(lfs, cfg);
    if (err) {
        return err;
    }

    // create free lookahead
    memset(lfs->free.buffer, 0, lfs->cfg->lookahead/8);
    lfs->free.off = 0;
    lfs->free.size = lfs_min(lfs->cfg->lookahead, lfs->cfg->block_count);
    lfs->free.i = 0;
    lfs_alloc_ack(lfs);

    // create superblock dir
    err = lfs_dir_alloc(lfs, &superdir);
    if (err) {
        return err;
    }

    // write root directory
    err = lfs_dir_alloc(lfs, &root);
    if (err) {
        return err;
    }

    err = lfs_dir_commit(lfs, &root, NULL, 0);
    if (err) {
        return err;
    }

    lfs->root[0] = root.pair[0];
    lfs->root[1] = root.pair[1];
    superdir.d.tail[0] = lfs->root[0];
    superdir.d.tail[1] = lfs->root[1];

    // write one superblock
    superblock.d.version = LFS_DISK_VERSION,
    superblock.d.root[0] = lfs->root[0];
    superblock.d.root[1] = lfs->root[1];
    superblock.d.block_size  = lfs->cfg->block_size;
    superblock.d.block_count = lfs->cfg->block_count;
    superblock.d.inline_size = lfs->inline_size;
    superblock.d.attrs_size  = lfs->attrs_size;
    superblock.d.name_size   = lfs->name_size;

    superentry.d.type = LFS_STRUCT_DIR | LFS_TYPE_SUPERBLOCK;
    superentry.d.elen = sizeof(superblock.d);
    superentry.d.alen = 0;
    superentry.d.nlen = strlen("littlefs");
    superentry.off = sizeof(superdir.d);
    superentry.size = 0;

    lfs_entry_tole32(&superentry.d);
    lfs_superblock_tole32(&superblock.d);
    err = lfs_dir_set(lfs, &superdir, &superentry, (struct lfs_region[]){
            {LFS_FROM_MEM, 0, 0, &superentry.d, 4},
            {LFS_FROM_MEM, 0, 0, &superblock.d, sizeof(superblock.d)},
            {LFS_FROM_MEM, 0, 0, "littlefs", superentry.d.nlen}}, 3);
    if (err) {
        return err;
    }

    // sanity check that fetch works
    err = lfs_dir_fetch(lfs, &superdir, (const lfs_block_t[2]){0, 1});
    if (err) {
        return err;
    }

    return lfs_deinit(lfs);
}

int lfs_mount(lfs_t *lfs, const struct lfs_config *cfg) {
    lfs_dir_t dir;
    lfs_superblock_t superblock;
    char magic[8];
    lfs_entry_t entry = {0};
    uint16_t major_version, minor_version;
    int err = lfs_init(lfs, cfg);
    if (err) {
        return err;
    }

    // setup free lookahead
    lfs->free.off = 0;
    lfs->free.size = 0;
    lfs->free.i = 0;
    lfs_alloc_ack(lfs);

    // load superblock
    err = lfs_dir_fetch(lfs, &dir, (const lfs_block_t[2]){0, 1});
    if (err) {
        if (err == LFS_ERR_CORRUPT) {
            LFS_ERROR("Invalid superblock at %d %d", 0, 1);
        }
        return err;
    }

    entry.off = sizeof(dir.d);
    err = lfs_dir_get(lfs, &dir, entry.off, &entry.d, 4);
    if (err) {
        return err;
    }

    memset(&superblock.d, 0, sizeof(superblock.d));
    err = lfs_dir_get(lfs, &dir,
            sizeof(dir.d)+4, &superblock.d,
            lfs_min(sizeof(superblock.d), lfs_entry_elen(&entry)));
    lfs_superblock_fromle32(&superblock.d);
    if (err) {
        return err;
    }

    err = lfs_dir_get(lfs, &dir,
            sizeof(dir.d)+lfs_entry_size(&entry)-entry.d.nlen, magic,
            lfs_min(sizeof(magic), entry.d.nlen));
    if (err) {
        return err;
    }

    if (memcmp(magic, "littlefs", 8) != 0) {
        LFS_ERROR("Invalid superblock at %d %d", 0, 1);
        return LFS_ERR_CORRUPT;
    }

    major_version = (0xffff & (superblock.d.version >> 16));
    minor_version = (0xffff & (superblock.d.version >>  0));
    if ((major_version != LFS_DISK_VERSION_MAJOR ||
         minor_version > LFS_DISK_VERSION_MINOR)) {
        LFS_ERROR("Invalid version %d.%d", major_version, minor_version);
        return LFS_ERR_INVAL;
    }

    if (superblock.d.inline_size) {
        if (superblock.d.inline_size > lfs->inline_size) {
            LFS_ERROR("Unsupported inline size (%d > %d)",
                    superblock.d.inline_size, lfs->inline_size);
            return LFS_ERR_INVAL;
        }

        lfs->inline_size = superblock.d.inline_size;
    }

    if (superblock.d.attrs_size) {
        if (superblock.d.attrs_size > lfs->attrs_size) {
            LFS_ERROR("Unsupported attrs size (%d > %d)",
                    superblock.d.attrs_size, lfs->attrs_size);
            return LFS_ERR_INVAL;
        }

        lfs->attrs_size = superblock.d.attrs_size;
    }

    if (superblock.d.name_size) {
        if (superblock.d.name_size > lfs->name_size) {
            LFS_ERROR("Unsupported name size (%d > %d)",
                    superblock.d.name_size, lfs->name_size);
            return LFS_ERR_INVAL;
        }

        lfs->name_size = superblock.d.name_size;
    }

    lfs->root[0] = superblock.d.root[0];
    lfs->root[1] = superblock.d.root[1];

    return 0;
}

int lfs_unmount(lfs_t *lfs) {
    return lfs_deinit(lfs);
}


/// Internal filesystem filesystem operations ///
int lfs_traverse(lfs_t *lfs, struct lfs_traverse_callbacks callbacks, void *data) {
    int i, err;
    lfs_file_t *f;
    lfs_dir_t dir;
    lfs_entry_t entry;
    lfs_block_t cwd[2] = {0, 1};

    if (lfs_pairisnull(lfs->root)) {
        return 0;
    }

    // iterate over metadata pairs

    while (true) {
        for (i = 0; i < 2; i++) {
            err = callbacks.block_callback(data, cwd[i]);
            if (err) {
                return err;
            }
        }

        err = lfs_dir_fetch(lfs, &dir, cwd);
        if (err) {
            return err;
        }

        // iterate over contents
        while (dir.off + sizeof(entry.d) <= (0x7fffffff & dir.d.size)-4) {
            err = lfs_dir_get(lfs, &dir,
                    dir.off, &entry.d, sizeof(entry.d));
            lfs_entry_fromle32(&entry.d);
            if (err) {
                return err;
            }

            entry.off = dir.off;
            entry.size = lfs_entry_size(&entry);

            if (callbacks.entry_callback) {
                callbacks.entry_callback(data, &dir, &entry);
            }

            dir.off += lfs_entry_size(&entry);
            if ((0x70 & entry.d.type) == LFS_STRUCT_CTZ) {
                err = lfs_ctz_traverse(lfs, &lfs->rcache, NULL,
                        entry.d.u.file.head, entry.d.u.file.size, callbacks.block_callback, data);
                if (err) {
                    return err;
                }
            }
        }

        cwd[0] = dir.d.tail[0];
        cwd[1] = dir.d.tail[1];

        if (lfs_pairisnull(cwd)) {
            break;
        }
    }

    // iterate over any open files
    for (f = lfs->files; f; f = f->next) {
        if ((f->flags & LFS_F_DIRTY) && !(f->flags & LFS_F_INLINE)) {
            err = lfs_ctz_traverse(lfs, &lfs->rcache, &f->cache,
                    f->head, f->size, callbacks.block_callback, data);
            if (err) {
                return err;
            }
        }

        if ((f->flags & LFS_F_WRITING) && !(f->flags & LFS_F_INLINE)) {
            err = lfs_ctz_traverse(lfs, &lfs->rcache, &f->cache,
                    f->block, f->pos, callbacks.block_callback, data);
            if (err) {
                return err;
            }
        }
    }

    return 0;
}

static int lfs_pred(lfs_t *lfs, const lfs_block_t dir[2], lfs_dir_t *pdir) {
    int err;
    if (lfs_pairisnull(lfs->root)) {
        return 0;
    }

    // iterate over all directory directory entries
    err = lfs_dir_fetch(lfs, pdir, (const lfs_block_t[2]){0, 1});
    if (err) {
        return err;
    }

    while (!lfs_pairisnull(pdir->d.tail)) {
        if (lfs_paircmp(pdir->d.tail, dir) == 0) {
            return true;
        }

        err = lfs_dir_fetch(lfs, pdir, pdir->d.tail);
        if (err) {
            return err;
        }
    }

    return false;
}

static int lfs_parent(lfs_t *lfs, const lfs_block_t dir[2],
        lfs_dir_t *parent, lfs_entry_t *entry) {
    if (lfs_pairisnull(lfs->root)) {
        return 0;
    }

    parent->d.tail[0] = 0;
    parent->d.tail[1] = 1;

    // iterate over all directory directory entries
    while (!lfs_pairisnull(parent->d.tail)) {
        int err = lfs_dir_fetch(lfs, parent, parent->d.tail);
        if (err) {
            return err;
        }

        while (true) {
            err = lfs_dir_next(lfs, parent, entry);
            if (err && err != LFS_ERR_NOENT) {
                return err;
            }

            if (err == LFS_ERR_NOENT) {
                break;
            }

            if (((0x70 & entry->d.type) == LFS_STRUCT_DIR) &&
                 lfs_paircmp(entry->d.u.dir, dir) == 0) {
                return true;
            }
        }
    }

    return false;
}

static int lfs_moved(lfs_t *lfs, const void *e) {
    lfs_dir_t cwd;
    lfs_entry_t entry;
    int err;
    if (lfs_pairisnull(lfs->root)) {
        return 0;
    }

    // skip superblock
    err = lfs_dir_fetch(lfs, &cwd, (const lfs_block_t[2]){0, 1});
    if (err) {
        return err;
    }

    // iterate over all directory directory entries
    while (!lfs_pairisnull(cwd.d.tail)) {
        err = lfs_dir_fetch(lfs, &cwd, cwd.d.tail);
        if (err) {
            return err;
        }

        while (true) {
            err = lfs_dir_next(lfs, &cwd, &entry);
            if (err && err != LFS_ERR_NOENT) {
                return err;
            }

            if (err == LFS_ERR_NOENT) {
                break;
            }

            if (!(LFS_STRUCT_MOVED & entry.d.type) &&
                 memcmp(&entry.d.u, e, sizeof(entry.d.u)) == 0) {
                return true;
            }
        }
    }

    return false;
}

static int lfs_relocate(lfs_t *lfs,
        const lfs_block_t oldpair[2], const lfs_block_t newpair[2]) {
    // find parent
    lfs_dir_t parent;
    lfs_entry_t entry;
    int err;
    int res = lfs_parent(lfs, oldpair, &parent, &entry);
    if (res < 0) {
        return res;
    }

    if (res) {
        // update disk, this creates a desync
        entry.d.u.dir[0] = newpair[0];
        entry.d.u.dir[1] = newpair[1];
        lfs_entry_tole32(&entry.d);
        err = lfs_dir_set(lfs, &parent, &entry, (struct lfs_region[]){
                {LFS_FROM_MEM, 0, sizeof(entry.d),
                    &entry.d, sizeof(entry.d)}}, 1);
        if (err) {
            return err;
        }

        // update internal root
        if (lfs_paircmp(oldpair, lfs->root) == 0) {
            LFS_DEBUG("Relocating root %d %d", newpair[0], newpair[1]);
            lfs->root[0] = newpair[0];
            lfs->root[1] = newpair[1];
        }

        // clean up bad block, which should now be a desync
        return lfs_deorphan(lfs);
    }

    // find pred
    res = lfs_pred(lfs, oldpair, &parent);
    if (res < 0) {
        return res;
    }

    if (res) {
        // just replace bad pair, no desync can occur
        parent.d.tail[0] = newpair[0];
        parent.d.tail[1] = newpair[1];

        return lfs_dir_commit(lfs, &parent, NULL, 0);
    }

    // couldn't find dir, must be new
    return 0;
}

int lfs_deorphan(lfs_t *lfs) {
    lfs_dir_t pdir = {.d.size = 0x80000000};
    lfs_dir_t cwd = {.d.tail[0] = 0, .d.tail[1] = 1};
    lfs_entry_t entry;

    lfs->deorphaned = true;

    if (lfs_pairisnull(lfs->root)) {
        return 0;
    }

    // iterate over all directory directory entries
    while (!lfs_pairisnull(cwd.d.tail)) {
        int err = lfs_dir_fetch(lfs, &cwd, cwd.d.tail);
        if (err) {
            return err;
        }

        // check head blocks for orphans
        if (!(0x80000000 & pdir.d.size)) {
            // check if we have a parent
            lfs_dir_t parent;
            lfs_entry_t entry;
            int res = lfs_parent(lfs, pdir.d.tail, &parent, &entry);
            if (res < 0) {
                return res;
            }

            if (!res) {
                // we are an orphan
                LFS_DEBUG("Found orphan %d %d",
                        pdir.d.tail[0], pdir.d.tail[1]);

                pdir.d.tail[0] = cwd.d.tail[0];
                pdir.d.tail[1] = cwd.d.tail[1];

                err = lfs_dir_commit(lfs, &pdir, NULL, 0);
                if (err) {
                    return err;
                }

                break;
            }

            if (!lfs_pairsync(entry.d.u.dir, pdir.d.tail)) {
                // we have desynced
                LFS_DEBUG("Found desync %d %d",
                        entry.d.u.dir[0], entry.d.u.dir[1]);

                pdir.d.tail[0] = entry.d.u.dir[0];
                pdir.d.tail[1] = entry.d.u.dir[1];

                err = lfs_dir_commit(lfs, &pdir, NULL, 0);
                if (err) {
                    return err;
                }

                break;
            }
        }

        // check entries for moves
        while (true) {
            err = lfs_dir_next(lfs, &cwd, &entry);
            if (err && err != LFS_ERR_NOENT) {
                return err;
            }

            if (err == LFS_ERR_NOENT) {
                break;
            }

            // found moved entry
            if (entry.d.type & LFS_STRUCT_MOVED) {
                int moved = lfs_moved(lfs, &entry.d.u);
                if (moved < 0) {
                    return moved;
                }

                if (moved) {
                    LFS_DEBUG("Found move %d %d",
                            entry.d.u.dir[0], entry.d.u.dir[1]);
                    err = lfs_dir_set(lfs, &cwd, &entry, (struct lfs_region[]){
                            {LFS_FROM_MEM, 0, entry.size, NULL, 0}}, 1);
                    if (err) {
                        return err;
                    }
                } else {
                    LFS_DEBUG("Found partial move %d %d",
                            entry.d.u.dir[0], entry.d.u.dir[1]);
                    entry.d.type &= ~LFS_STRUCT_MOVED;
                    err = lfs_dir_set(lfs, &cwd, &entry, (struct lfs_region[]){
                            {LFS_FROM_MEM, 0, 1, &entry.d, 1}}, 1);
                    if (err) {
                        return err;
                    }
                }
            }
        }

        memcpy(&pdir, &cwd, sizeof(pdir));
    }

    return 0;
}


/// External filesystem filesystem operations ///
int lfs_fs_getattrs(lfs_t *lfs, const struct lfs_attr *attrs, int count) {
    lfs_dir_t dir;
    lfs_entry_t entry = {0};
    int err = lfs_dir_fetch(lfs, &dir, (const lfs_block_t[2]){0, 1});
    if (err) {
        return err;
    }

    entry.off = sizeof(dir.d);
    err = lfs_dir_get(lfs, &dir, entry.off, &entry.d, 4);
    if (err) {
        return err;
    }
    entry.size = lfs_entry_size(&entry);

    return lfs_dir_getattrs(lfs, &dir, &entry, attrs, count);
}

int lfs_fs_setattrs(lfs_t *lfs, const struct lfs_attr *attrs, int count) {
    lfs_dir_t dir;
    lfs_entry_t entry = {0};
    int err = lfs_dir_fetch(lfs, &dir, (const lfs_block_t[2]){0, 1});
    if (err) {
        return err;
    }

    entry.off = sizeof(dir.d);
    err = lfs_dir_get(lfs, &dir, entry.off, &entry.d, 4);
    if (err) {
        return err;
    }
    entry.size = lfs_entry_size(&entry);

    return lfs_dir_setattrs(lfs, &dir, &entry, attrs, count);
}

static int lfs_fs_size_count(void *p, lfs_block_t block) {
    lfs_size_t *size = p;
    *size += 1;
    return 0;
}

lfs_ssize_t lfs_fs_size(lfs_t *lfs) {
    lfs_size_t size = 0;
    struct lfs_traverse_callbacks callbacks = {
        .block_callback = lfs_fs_size_count,
        .entry_callback = NULL
    };
    int err = lfs_traverse(lfs, callbacks, &size);
    if (err) {
        return err;
    }

    return size;
}
