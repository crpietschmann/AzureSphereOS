FILESEXTRAPATHS_prepend := "${THISDIR}/gdbserver-files:${THISDIR}/files:"

SRC_URI += "file://0012-Implement-vFile-buildid-in-hostio.patch \
            file://0013-reduce-copy-buffer-size.patch \
            file://0014-Don-t-use-large-fixed-size-buffers-for-paths.patch \
            file://0015-Change-startup-with-shell-to-not-be-the-default.patch \
            file://0016-Do-not-call-setpgid.patch \
            file://app_manifest.json \
           "

GDB_IMAGE_PACKGE_DIR = "gdbserver_files"

# Package up the output needed to create an installable gdbserver app
FILES_${PN} += "\
                ${exec_prefix}/${GDB_IMAGE_PACKGE_DIR}/bin/gdbserver     \
                ${exec_prefix}/${GDB_IMAGE_PACKGE_DIR}/app_manifest.json \
               "

APPMANIFEST = "${WORKDIR}/app_manifest.json"

# We only need GDB server on the target - so we disable dependencies and builds of other tools
DEPENDS_class-target = ""
PACKAGECONFIG_class-target = ""
EXTRA_OECONF_class-target = "--disable-gdbtk --disable-tui --disable-x --disable-werror \
							--without-ncurses --disable-multilib --disable-sim \
							--without-lzma --without-guile \
							${GDBPROPREFIX} --without-expat \
							--disable-rpath --without-zlib \
							"

# Don't use the stat replacement
EXTRA_OEMAKE_append_libc-musl = "\
                                gl_cv_func_stat_dir_slash=yes \
                                gl_cv_func_stat_file_slash=yes \
                                "

EXTRA_OEMAKE_class-target = "'SUBDIRS=gdb/gdbserver'"

do_install_class-target() {
        cd ${B}/gdb/gdbserver
        oe_runmake DESTDIR=${D} install

        rm -rf ${D}${libdir}
        rm -rf ${D}${includedir}
        rm -rf ${D}${datadir}/locale

        # Move gdbserver from bin to gdbserver_files/bin/
        mkdir ${D}/usr/${GDB_IMAGE_PACKGE_DIR}
        mv ${D}/usr/bin ${D}/usr/${GDB_IMAGE_PACKGE_DIR}

        # Install app manifest
        install -m 0644 ${APPMANIFEST} ${D}/usr/${GDB_IMAGE_PACKGE_DIR}
}
