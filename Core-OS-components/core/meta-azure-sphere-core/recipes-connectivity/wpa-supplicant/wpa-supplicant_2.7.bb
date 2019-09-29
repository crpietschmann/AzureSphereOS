SUMMARY = "Client for Wi-Fi Protected Access (WPA)"
HOMEPAGE = "http://w1.fi/wpa_supplicant/"
BUGTRACKER = "http://w1.fi/security/"
SECTION = "network"
LICENSE = "BSD"
LIC_FILES_CHKSUM = "file://COPYING;md5=a3791c270ad6bb026707d17bf750e5ef \
                    file://README;beginline=1;endline=56;md5=495cbce6008253de4b4d8f4cdfae9f4f \
                    file://wpa_supplicant/wpa_supplicant.c;beginline=1;endline=12;md5=a5687903a31b8679e6a06b3afa5c819e"

DEPENDS = "dbus libnl"
RRECOMMENDS_${PN} = "wpa-supplicant-cli"

PACKAGECONFIG ??= ""
PACKAGECONFIG[gnutls] = ",,gnutls libgcrypt"
PACKAGECONFIG[openssl] = ",,openssl"

inherit pkgconfig systemd

SYSTEMD_SERVICE_${PN} = "wpa_supplicant.service wpa_supplicant-nl80211@.service wpa_supplicant-wired@.service"
SYSTEMD_AUTO_ENABLE = "disable"

SRC_URI = "http://w1.fi/releases/wpa_supplicant-${PV}.tar.gz  \
           file://defconfig \
          "
SRC_URI[md5sum] = "a68538fb62766f40f890125026c42c10"
SRC_URI[sha256sum] = "76ea6b06b7a2ea8e6d9eb1a9166166f1656e6d48c7508914f592100c95c73074"

CVE_PRODUCT = "wpa_supplicant"

S = "${WORKDIR}/wpa_supplicant-${PV}"

PACKAGES_prepend = "wpa-supplicant-cli "
FILES_wpa-supplicant-cli = "${sbindir}/wpa_cli"

do_configure () {
	${MAKE} -C wpa_supplicant clean
	install -m 0755 ${WORKDIR}/defconfig wpa_supplicant/.config
	echo "CFLAGS +=\"-I${STAGING_INCDIR}/libnl3\"" >> wpa_supplicant/.config

	# Don't write blobs back to configuration file when saving configs. Otherwise, we end up
	# with all the blobs written to wpa_supplicant.conf
	echo "CFLAGS +=\"-DCONFIG_DONT_WRITE_BLOBS\"" >> wpa_supplicant/.config

	# Allow wolfSSL_CTX_load_verify_buffer to succeed even if the date on the
	# CA root certificate is invalid according to the device's clock.
	echo "CFLAGS +=\"-DCONFIG_AZSPHERE_DISABLE_TIME_CHECKS\"" >> wpa_supplicant/.config

	echo "DRV_CFLAGS +=\"-I${STAGING_INCDIR}/libnl3\"" >> wpa_supplicant/.config
	if [ -z ${ssl+1} ]; then
			ssl=internal
	fi
	if echo "${PACKAGECONFIG}" | grep -qw "openssl"; then
        	ssl=openssl
	elif echo "${PACKAGECONFIG}" | grep -qw "gnutls"; then
        	ssl=gnutls
	fi
	if [ -n "$ssl" ]; then
        	sed -i "s/%ssl%/$ssl/" wpa_supplicant/.config
	fi

	# For rebuild
	rm -f wpa_supplicant/*.d wpa_supplicant/dbus/*.d
}

export EXTRA_CFLAGS = "${CFLAGS}"
export BINDIR = "${sbindir}"

do_compile () {
	unset CFLAGS CPPFLAGS CXXFLAGS
	sed -e "s:CFLAGS\ =.*:& \$(EXTRA_CFLAGS):g" -i ${S}/src/lib.rules
	oe_runmake -C wpa_supplicant
}

do_install () {
	install -d ${D}${sbindir}
	install -m 755 wpa_supplicant/wpa_supplicant ${D}${sbindir}
	install -m 755 wpa_supplicant/wpa_cli        ${D}${sbindir}
}

pkg_postinst_wpa-supplicant () {
	# If we're offline, we don't need to do this.
	if [ "x$D" = "x" ]; then
		killall -q -HUP dbus-daemon || true
	fi

}
