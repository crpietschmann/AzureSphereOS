#
# Azure Sphere specific WPA Supplicant tweaks
#

# Use local files
FILESEXTRAPATHS_prepend := "${THISDIR}/files:"

# SSL config (use built in TLS instead of pulling in OpenSSL)
PACKAGECONFIG = "internal"
PACKAGECONFIG[internal] = ""

# Exclude dbus since it's not supported on Azure Sphere
DEPENDS_remove = "dbus"

# The authors have indicated this CVE is not applicable
CVE_CHECK_CVE_WHITELIST = "{\
    'CVE-2017-13084': ('2.6',), \
}"


# Pull in patches
SRC_URI += "file://wpa_ctrl_groups.patch \
            file://rebased-v2.6-0001-WPA-Ignore-unauthenticated-encrypted-EAPOL-Key-data.patch"
