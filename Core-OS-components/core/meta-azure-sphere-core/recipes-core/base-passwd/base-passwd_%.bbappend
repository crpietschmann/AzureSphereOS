FILESEXTRAPATHS_prepend := "${THISDIR}/base-passwd:"

SRC_URI += "file://passwd.azuresphere.master \
            file://group.azuresphere.master"

do_install_append () {
    # Replace the built group / passwd with custom list
	install -o root -g root -p -m 644 ${WORKDIR}/passwd.azuresphere.master ${D}${datadir}/base-passwd/passwd.master
	install -o root -g root -p -m 644 ${WORKDIR}/group.azuresphere.master ${D}${datadir}/base-passwd/group.master
}