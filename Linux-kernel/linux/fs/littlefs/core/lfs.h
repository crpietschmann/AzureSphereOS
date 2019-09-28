/*
 * The little filesystem
 *
 * Copyright (c) 2017, Arm Limited. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef LFS_H
#define LFS_H

#include <linux/types.h>

/// Version info ///

// Software library version
// Major (top-nibble), incremented on backwards incompatible changes
// Minor (bottom-nibble), incremented on feature additions
#define LFS_VERSION 0x00010004
#define LFS_VERSION_MAJOR (0xffff & (LFS_VERSION >> 16))
#define LFS_VERSION_MINOR (0xffff & (LFS_VERSION >>  0))

// Version of On-disk data structures
// Major (top-nibble), incremented on backwards incompatible changes
// Minor (bottom-nibble), incremented on feature additions
#define LFS_DISK_VERSION 0x00010002
#define LFS_DISK_VERSION_MAJOR (0xffff & (LFS_DISK_VERSION >> 16))
#define LFS_DISK_VERSION_MINOR (0xffff & (LFS_DISK_VERSION >>  0))


/// Definitions ///

// Type definitions
typedef uint32_t lfs_size_t;
typedef uint32_t lfs_off_t;

typedef int32_t  lfs_ssize_t;
typedef int32_t  lfs_soff_t;

typedef uint32_t lfs_block_t;

// Maximum inline file size in bytes. Large inline files require a larger
// read and prog cache, but if a file can be inline it does not need its own
// data block. LFS_ATTRS_MAX + LFS_INLINE_MAX must be <= 0xffff. Stored in
// superblock and must be respected by other littlefs drivers.
#ifndef LFS_INLINE_MAX
#define LFS_INLINE_MAX 0x3ff
#endif

// Maximum size of all attributes per file in bytes, may be redefined but a
// a smaller LFS_ATTRS_MAX has no benefit. LFS_ATTRS_MAX + LFS_INLINE_MAX
// must be <= 0xffff. Stored in superblock and must be respected by other
// littlefs drivers.
#ifndef LFS_ATTRS_MAX
#define LFS_ATTRS_MAX 0x3f
#endif

// Max name size in bytes, may be redefined to reduce the size of the
// info struct. Stored in superblock and must be respected by other
// littlefs drivers.
#ifndef LFS_NAME_MAX
#define LFS_NAME_MAX 0xff
#endif

// Possible error codes, these are negative to allow
// valid positive return values
enum lfs_error {
    LFS_ERR_OK          = 0,    // No error
    LFS_ERR_IO          = -5,   // Error during device operation
    LFS_ERR_CORRUPT     = -52,  // Corrupted
    LFS_ERR_NOENT       = -2,   // No directory entry
    LFS_ERR_EXIST       = -17,  // Entry already exists
    LFS_ERR_NOTDIR      = -20,  // Entry is not a dir
    LFS_ERR_ISDIR       = -21,  // Entry is a dir
    LFS_ERR_NOTEMPTY    = -39,  // Dir is not empty
    LFS_ERR_BADF        = -9,   // Bad file number
    LFS_ERR_INVAL       = -22,  // Invalid parameter
    LFS_ERR_NOSPC       = -28,  // No space left on device
    LFS_ERR_NOMEM       = -12,  // No more memory available
    LFS_ERR_NAMETOOLONG = -36,  // File name too long
    LFS_ERR_NODATA      = -61,  // No data/attr available
    LFS_ERR_RANGE       = -34,  // Result not representable
};

// File types
enum lfs_type {
    // file type
    LFS_TYPE_REG        = 0x01,
    LFS_TYPE_DIR        = 0x02,
    LFS_TYPE_SUPERBLOCK = 0x0e,

    // on disk structure
    LFS_STRUCT_CTZ      = 0x10,
    LFS_STRUCT_DIR      = 0x20,
    LFS_STRUCT_INLINE   = 0x30,
    LFS_STRUCT_MOVED    = 0x80,
};

// File open flags
enum lfs_open_flags {
    // open flags
    LFS_O_RDONLY = 1,         // Open a file as read only
    LFS_O_WRONLY = 2,         // Open a file as write only
    LFS_O_RDWR   = 3,         // Open a file as read and write
    LFS_O_CREAT  = 0x0100,    // Create a file if it does not exist
    LFS_O_EXCL   = 0x0200,    // Fail if a file already exists
    LFS_O_TRUNC  = 0x0400,    // Truncate the existing file to zero size
    LFS_O_APPEND = 0x0800,    // Move to end of file on every write

    // internally used flags
    LFS_F_DIRTY   = 0x010000, // File does not match storage
    LFS_F_WRITING = 0x020000, // File has been written since last flush
    LFS_F_READING = 0x040000, // File has been read since last flush
    LFS_F_ERRED   = 0x080000, // An error occured during write
    LFS_F_INLINE  = 0x100000, // Currently inlined in directory entry
};

// File seek flags
enum lfs_whence_flags {
    LFS_SEEK_SET = 0,   // Seek relative to an absolute position
    LFS_SEEK_CUR = 1,   // Seek relative to the current file position
    LFS_SEEK_END = 2,   // Seek relative to the end of the file
};


// Configuration provided during initialization of the littlefs
struct lfs_config {
    // Opaque user provided context that can be used to pass
    // information to the block device operations
    void *context;

    // Read a region in a block. Negative error codes are propogated
    // to the user.
    int (*read)(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, void *buffer, lfs_size_t size);

    // Program a region in a block. The block must have previously
    // been erased. Negative error codes are propogated to the user.
    // May return LFS_ERR_CORRUPT if the block should be considered bad.
    int (*prog)(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, const void *buffer, lfs_size_t size);

    // Erase a block. A block must be erased before being programmed.
    // The state of an erased block is undefined. Negative error codes
    // are propogated to the user.
    // May return LFS_ERR_CORRUPT if the block should be considered bad.
    int (*erase)(const struct lfs_config *c, lfs_block_t block);

    // Sync the state of the underlying block device. Negative error codes
    // are propogated to the user.
    int (*sync)(const struct lfs_config *c);

    // Minimum size of a block read. This determines the size of read buffers.
    // This may be larger than the physical read size to improve performance
    // by caching more of the block device.
    lfs_size_t read_size;

    // Minimum size of a block program. This determines the size of program
    // buffers. This may be larger than the physical program size to improve
    // performance by caching more of the block device.
    // Must be a multiple of the read size.
    lfs_size_t prog_size;

    // Size of an erasable block. This does not impact ram consumption and
    // may be larger than the physical erase size. However, this should be
    // kept small as each file currently takes up an entire block.
    // Must be a multiple of the program size.
    lfs_size_t block_size;

    // Number of erasable blocks on the device.
    lfs_size_t block_count;

    // Number of blocks to lookahead during block allocation. A larger
    // lookahead reduces the number of passes required to allocate a block.
    // The lookahead buffer requires only 1 bit per block so it can be quite
    // large with little ram impact. Should be a multiple of 32.
    lfs_size_t lookahead;

    // Optional, statically allocated read buffer. Must be read sized.
    void *read_buffer;

    // Optional, statically allocated program buffer. Must be program sized.
    void *prog_buffer;

    // Optional, statically allocated lookahead buffer. Must be 1 bit per
    // lookahead block.
    void *lookahead_buffer;

    // Optional, statically allocated buffer for files. Must be program sized.
    // If enabled, only one file may be opened at a time.
    void *file_buffer;

    // Optional upper limit on inlined files in bytes. Large inline files
    // require a larger read and prog cache, but if a file can be inlined it
    // does not need its own data block. Must be smaller than the read size
    // and prog size. Defaults to min(LFS_INLINE_MAX, read_size) when zero.
    // Stored in superblock and must be respected by other littlefs drivers.
    lfs_size_t inline_size;

    // Optional upper limit on attributes per file in bytes. No downside for
    // larger attributes size but must be less than LFS_ATTRS_MAX. Defaults to
    // LFS_ATTRS_MAX when zero.Stored in superblock and must be respected by
    // other littlefs drivers.
    lfs_size_t attrs_size;

    // Optional upper limit on length of file names in bytes. No downside for
    // larger names except the size of the info struct which is controlled by
    // the LFS_NAME_MAX define. Defaults to LFS_NAME_MAX when zero. Stored in
    // superblock and must be respected by other littlefs drivers.
    lfs_size_t name_size;
};


// File info structure
struct lfs_info {
    // Type of the file, either LFS_TYPE_REG or LFS_TYPE_DIR
    uint8_t type;

    // Size of the file, only valid for REG files
    lfs_size_t size;

    // Name of the file stored as a null-terminated string
    char name[LFS_NAME_MAX+1];
};

// Custom attribute structure
struct lfs_attr {
    // Type of attribute, provided by user and used to identify the attribute
    uint8_t type;

    // Pointer to buffer containing the attribute
    void *buffer;

    // Size of attribute in bytes, limited to LFS_ATTRS_MAX
    lfs_size_t size;
};


/// littlefs data structures ///
typedef struct lfs_entry {
    lfs_off_t off;
    lfs_size_t size;

    struct lfs_disk_entry {
        uint8_t type;
        uint8_t elen;
        uint8_t alen;
        uint8_t nlen;
        union {
            struct {
                lfs_block_t head;
                lfs_size_t size;
            } file;
            lfs_block_t dir[2];
        } u;
    } d;
} lfs_entry_t;

typedef struct lfs_entry_attr {
    struct lfs_disk_entry_attr {
        uint8_t type;
        uint8_t len;
    } d;
} lfs_entry_attr_t;

typedef struct lfs_cache {
    lfs_block_t block;
    lfs_off_t off;
    uint8_t *buffer;
} lfs_cache_t;

typedef struct lfs_file {
    struct lfs_file *next;
    lfs_block_t pair[2];
    lfs_off_t pairoff;

    lfs_block_t head;
    lfs_size_t size;

    uint32_t flags;
    lfs_size_t inline_size;
    lfs_off_t pos;
    lfs_block_t block;
    lfs_off_t off;
    lfs_cache_t cache;

    const struct lfs_attr *attrs;
    int attrcount;
} lfs_file_t;

typedef struct lfs_dir {
    struct lfs_dir *next;
    lfs_block_t pair[2];
    lfs_off_t off;

    lfs_block_t head[2];
    lfs_off_t pos;

    struct lfs_disk_dir {
        uint32_t rev;
        lfs_size_t size;
        lfs_block_t tail[2];
    } d;
} lfs_dir_t;

typedef struct lfs_superblock {
    struct lfs_disk_superblock {
        lfs_block_t root[2];

        lfs_size_t block_size;
        lfs_size_t block_count;
        uint32_t version;

        lfs_size_t inline_size;
        lfs_size_t attrs_size;
        lfs_size_t name_size;
    } d;
} lfs_superblock_t;

typedef struct lfs_free {
    lfs_block_t off;
    lfs_block_t size;
    lfs_block_t i;
    lfs_block_t ack;
    uint32_t *buffer;
} lfs_free_t;

// The littlefs type
typedef struct lfs {
    const struct lfs_config *cfg;

    lfs_block_t root[2];
    lfs_file_t *files;
    lfs_dir_t *dirs;

    lfs_cache_t rcache;
    lfs_cache_t pcache;

    lfs_free_t free;
    bool deorphaned;

    lfs_size_t inline_size;
    lfs_size_t attrs_size;
    lfs_size_t name_size;
} lfs_t;

struct lfs_traverse_callbacks {
    int (*block_callback)(void*, lfs_block_t);
    int (*entry_callback)(void*, lfs_dir_t*, lfs_entry_t*);
};

/// Filesystem functions ///

// Format a block device with the littlefs
//
// Requires a littlefs object and config struct. This clobbers the littlefs
// object, and does not leave the filesystem mounted.
//
// Returns a negative error code on failure.
int lfs_format(lfs_t *lfs, const struct lfs_config *config);

// Mounts a littlefs
//
// Requires a littlefs object and config struct. Multiple filesystems
// may be mounted simultaneously with multiple littlefs objects. Both
// lfs and config must be allocated while mounted.
//
// Returns a negative error code on failure.
int lfs_mount(lfs_t *lfs, const struct lfs_config *config);

// Unmounts a littlefs
//
// Does nothing besides releasing any allocated resources.
// Returns a negative error code on failure.
int lfs_unmount(lfs_t *lfs);

/// General operations ///

// Removes a file or directory
//
// If removing a directory, the directory must be empty.
// Returns a negative error code on failure.
int lfs_remove(lfs_t *lfs, const char *path);

// Rename or move a file or directory
//
// If the destination exists, it must match the source in type.
// If the destination is a directory, the directory must be empty.
//
// Returns a negative error code on failure.
int lfs_rename(lfs_t *lfs, const char *oldpath, const char *newpath);

// Find info about a file or directory
//
// Fills out the info structure, based on the specified file or directory.
// Returns a negative error code on failure.
int lfs_stat(lfs_t *lfs, const char *path, struct lfs_info *info);

// Get custom attributes
//
// Attributes are looked up based on the type id. If the stored attribute is
// smaller than the buffer, it is padded with zeros. It the stored attribute
// is larger than the buffer, LFS_ERR_RANGE is returned.
//
// Returns a negative error code on failure.
int lfs_getattrs(lfs_t *lfs, const char *path,
        const struct lfs_attr *attrs, int count);

// Set custom attributes
//
// The array of attributes will be used to update the attributes stored on
// disk based on their type id. Unspecified attributes are left unmodified.
// Specifying an attribute with zero size deletes the attribute.
//
// Returns a negative error code on failure.
int lfs_setattrs(lfs_t *lfs, const char *path,
        const struct lfs_attr *attrs, int count);


/// File operations ///

// Open a file
//
// The mode that the file is opened in is determined
// by the flags, which are values from the enum lfs_open_flags
// that are bitwise-ored together.
//
// Returns a negative error code on failure.
int lfs_file_open(lfs_t *lfs, lfs_file_t *file,
        const char *path, int flags);

// Close a file
//
// Any pending writes are written out to storage as though
// sync had been called and releases any allocated resources.
//
// Returns a negative error code on failure.
int lfs_file_close(lfs_t *lfs, lfs_file_t *file);

// Synchronize a file on storage
//
// Any pending writes are written out to storage.
// Returns a negative error code on failure.
int lfs_file_sync(lfs_t *lfs, lfs_file_t *file);

// Read data from file
//
// Takes a buffer and size indicating where to store the read data.
// Returns the number of bytes read, or a negative error code on failure.
lfs_ssize_t lfs_file_read(lfs_t *lfs, lfs_file_t *file,
        void *buffer, lfs_size_t size);

// Write data to file
//
// Takes a buffer and size indicating the data to write. The file will not
// actually be updated on the storage until either sync or close is called.
//
// Returns the number of bytes written, or a negative error code on failure.
lfs_ssize_t lfs_file_write(lfs_t *lfs, lfs_file_t *file,
        const void *buffer, lfs_size_t size);

// Change the position of the file
//
// The change in position is determined by the offset and whence flag.
// Returns the old position of the file, or a negative error code on failure.
lfs_soff_t lfs_file_seek(lfs_t *lfs, lfs_file_t *file,
        lfs_soff_t off, int whence);

// Truncates the size of the file to the specified size
//
// Returns a negative error code on failure.
int lfs_file_truncate(lfs_t *lfs, lfs_file_t *file, lfs_off_t size);

// Return the position of the file
//
// Equivalent to lfs_file_seek(lfs, file, 0, LFS_SEEK_CUR)
// Returns the position of the file, or a negative error code on failure.
lfs_soff_t lfs_file_tell(lfs_t *lfs, lfs_file_t *file);

// Change the position of the file to the beginning of the file
//
// Equivalent to lfs_file_seek(lfs, file, 0, LFS_SEEK_CUR)
// Returns a negative error code on failure.
int lfs_file_rewind(lfs_t *lfs, lfs_file_t *file);

// Return the size of the file
//
// Similar to lfs_file_seek(lfs, file, 0, LFS_SEEK_END)
// Returns the size of the file, or a negative error code on failure.
lfs_soff_t lfs_file_size(lfs_t *lfs, lfs_file_t *file);

// Get custom attributes attached to a file
//
// Attributes are looked up based on the type id. If the stored attribute is
// smaller than the buffer, it is padded with zeros. It the stored attribute
// is larger than the buffer, LFS_ERR_RANGE is returned.
//
// Returns a negative error code on failure.
int lfs_file_getattrs(lfs_t *lfs, lfs_file_t *file,
        const struct lfs_attr *attrs, int count);

// Set custom attributes on a file
//
// The array of attributes will be used to update the attributes stored on
// disk based on their type id. Unspecified attributes are left unmodified.
// Specifying an attribute with zero size deletes the attribute.
//
// Note: Attributes are not written out until a call to lfs_file_sync
// or lfs_file_close and must be allocated until the file is closed or
// lfs_file_setattrs is called with a count of zero.
//
// Returns a negative error code on failure.
int lfs_file_setattrs(lfs_t *lfs, lfs_file_t *file,
        const struct lfs_attr *attrs, int count);


/// Directory operations ///

// Create a directory
//
// Returns a negative error code on failure.
int lfs_mkdir(lfs_t *lfs, const char *path);

// Open a directory
//
// Once open a directory can be used with read to iterate over files.
// Returns a negative error code on failure.
int lfs_dir_open(lfs_t *lfs, lfs_dir_t *dir, const char *path);

// Close a directory
//
// Releases any allocated resources.
// Returns a negative error code on failure.
int lfs_dir_close(lfs_t *lfs, lfs_dir_t *dir);

// Read an entry in the directory
//
// Fills out the info structure, based on the specified file or directory.
// Returns a negative error code on failure.
int lfs_dir_read(lfs_t *lfs, lfs_dir_t *dir, struct lfs_info *info);

// Change the position of the directory
//
// The new off must be a value previous returned from tell and specifies
// an absolute offset in the directory seek.
//
// Returns a negative error code on failure.
int lfs_dir_seek(lfs_t *lfs, lfs_dir_t *dir, lfs_off_t off);

// Return the position of the directory
//
// The returned offset is only meant to be consumed by seek and may not make
// sense, but does indicate the current position in the directory iteration.
//
// Returns the position of the directory, or a negative error code on failure.
lfs_soff_t lfs_dir_tell(lfs_t *lfs, lfs_dir_t *dir);

// Change the position of the directory to the beginning of the directory
//
// Returns a negative error code on failure.
int lfs_dir_rewind(lfs_t *lfs, lfs_dir_t *dir);


/// Filesystem filesystem operations ///

// Get custom attributes on the filesystem
//
// Attributes are looked up based on the type id. If the stored attribute is
// smaller than the buffer, it is padded with zeros. It the stored attribute
// is larger than the buffer, LFS_ERR_RANGE is returned.
//
// Returns a negative error code on failure.
int lfs_fs_getattrs(lfs_t *lfs, const struct lfs_attr *attrs, int count);

// Set custom attributes on the filesystem
//
// The array of attributes will be used to update the attributes stored on
// disk based on their type id. Unspecified attributes are left unmodified.
// Specifying an attribute with zero size deletes the attribute.
//
// Note: Filesystem level attributes are not available for wear-leveling
//
// Returns a negative error code on failure.
int lfs_fs_setattrs(lfs_t *lfs, const struct lfs_attr *attrs, int count);

// Finds the current size of the filesystem
//
// Note: Result is best effort. If files share COW structures, the returned
// size may be larger than the filesystem actually is.
//
// Returns the number of allocated blocks, or a negative error code on failure.
lfs_ssize_t lfs_fs_size(lfs_t *lfs);


/// Miscellaneous littlefs specific operations ///

// Traverse through all blocks in use by the filesystem
//
// The provided callback will be called with each block address that is
// currently in use by the filesystem. This can be used to determine which
// blocks are in use or how much of the storage is available.
//
// Returns a negative error code on failure.
int lfs_traverse(lfs_t *lfs, struct lfs_traverse_callbacks, void *data);

// Prunes any recoverable errors that may have occured in the filesystem
//
// Not needed to be called by user unless an operation is interrupted
// but the filesystem is still mounted. This is already called on first
// allocation.
//
// Returns a negative error code on failure.
int lfs_deorphan(lfs_t *lfs);

// Make this public so we can walk entries for quota data
int lfs_dir_getattrs(lfs_t *lfs, lfs_dir_t *dir, const lfs_entry_t *entry, const struct lfs_attr *attrs, int count);

#endif
