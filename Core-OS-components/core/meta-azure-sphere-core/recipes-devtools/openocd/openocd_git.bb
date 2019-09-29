DESCRIPTION = "OpenOCD"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=b234ee4d69f5fce4486a80fdaf4a4263"

inherit autotools pkgconfig

DEPENDS = "libusb1"

SRC_URI = "git://repo.or.cz/openocd.git;protocol=http;name=openocd \
           file://0001-MT3620-support.patch \
           file://0002-CVE-2018-5704-fix.patch \
           file://0003-Disable-noisy-log-line.patch \
           file://mt3620-rdb-ftdi.cfg \
           file://mt3620-io0.cfg \
           file://mt3620-io1.cfg \
           "

# Master commit post 0.10.0
PV = "0.10.0+git${SRCPV}"
SRCREV = "6ea43726a801baa718fd08dcdb8ae5835b8a2385"

BBCLASSEXTEND = "native nativesdk"

# Already fixed in the master build we're picking up
CVE_CHECK_CVE_WHITELIST = "{\
    'CVE-2018-5704': ('0.10.0',), \
}"

S = "${WORKDIR}/git"

# OpenOCD requires to build in tree
B = "${S}"

# Disable some warnings that the code will trip over in GCC7+
CPPFLAGS_append = " -Wimplicit-fallthrough=0 -Wno-format-truncation -Wno-format-overflow -Wno-tautological-compare "

do_configure() {
    ${S}/bootstrap

    oe_runconf
}

do_install_append() {
    # Add scripts for MT3620
    install -m 444 ${WORKDIR}/mt3620-rdb-ftdi.cfg ${D}${datadir}/openocd/scripts/
    install -m 444 ${WORKDIR}/mt3620-io0.cfg ${D}${datadir}/openocd/scripts/
    install -m 444 ${WORKDIR}/mt3620-io1.cfg ${D}${datadir}/openocd/scripts/
}