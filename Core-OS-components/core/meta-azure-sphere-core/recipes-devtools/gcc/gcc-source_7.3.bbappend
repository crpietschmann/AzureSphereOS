# Pull in the arena size patch, target only
FILESEXTRAPATHS_prepend := "${THISDIR}/gcc-7.3:"

SRC_URI_append_class-target += "\
    file://0049-Set-pool-size-to-zero.patch \
    file://0050-Hide-demangle-symbols.patch \
    "