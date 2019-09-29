
FILESEXTRAPATHS_prepend := "${THISDIR}/base-files:"

# Remove folders not used in Azure Sphere
dirs555_remove = "/sys"
dirs755_remove = "/boot"
dirs755_remove = "${base_bindir}"
dirs755_remove = "${base_sbindir}"
dirs755_remove = "${ROOT_HOME}"
dirs755_remove = "/usr/games"
dirs755_remove = "${includedir}"
dirs755_remove = "${mandir}"
dirs755_remove = "${datadir}/dict"
dirs755_remove = "${datadir}/misc"
dirs755_remove = "${datadir}"
dirs755_remove = "${infodir}"
dirs755_remove = "/sys"
dirs755_remove = "/home"
dirs755_remove = "/media"
dirs755_remove = "${prefix}/src"
dirs755_remove = "${localstatedir}/backups"
dirs755_remove = "${localstatedir}/lib"
dirs755_remove = "${localstatedir}/lib/misc"
dirs755_remove = "${localstatedir}/spool"
dirs755_remove = "${localstatedir}/volatile/log"
dirs755_remove = " ${localstatedir}/local"

volatiles = "tmp"

BASEFILESISSUEINSTALL=""

do_install_append() {
    # Remove shell profiles
    rm -rf ${D}${sysconfdir}/skel

    # Remove folders that have files in them installed by this recipe
    rm -rf ${D}${sysconfdir}/default

    # remove links not used
    rm ${D}${localstatedir}/lock

    # Remove misc files
	rm ${D}${sysconfdir}/filesystems
	rm ${D}${sysconfdir}/profile
	rm ${D}${sysconfdir}/shells
	rm ${D}${sysconfdir}/nsswitch.conf
	rm ${D}${sysconfdir}/host.conf
	rm ${D}${sysconfdir}/motd
}