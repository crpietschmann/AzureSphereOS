#
# Expose libbacktrace for usage elsewhere
#
RUNTIMETARGET_append = " libbacktrace"

do_install_append() {
    # make install doesn't copy libbacktrace as its a build but not output artifact
    install -m 0644 ${B}/${TARGET_SYS}/libbacktrace/.libs/libbacktrace.a ${D}${libdir}/libbacktrace.a
    install -m 0644 ${S}/libbacktrace/backtrace.h ${D}${includedir}/c++/${PV}/backtrace.h
}

FILES_libstdc++-dev_append = "\
    ${libdir}/libbacktrace.la \
    ${includedir}/c++/${PV}/backtrace.h \
"
FILES_libstdc++-staticdev_append = "\
    ${libdir}/libbacktrace.a \
"