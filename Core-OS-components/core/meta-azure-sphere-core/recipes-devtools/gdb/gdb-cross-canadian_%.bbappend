# Add patches for the SDK build of GDB
CANADIANEXTRAOS=""

FILESEXTRAPATHS_prepend := "${THISDIR}/files:"

SRC_URI += "file://0001-Used-cached-reads-for-reading-code-locations-during-.patch \
            file://0002-Implement-local-solib-caching.patch \
	    file://0003-Only-fetch-values-for-children-actually-requested.patch \
"
