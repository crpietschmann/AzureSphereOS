#
# Azure Sphere specific WPA Supplicant tweaks
#

# Use local files
FILESEXTRAPATHS_prepend := "${THISDIR}/files:"

# Exclude dbus since it's not supported on Azure Sphere
DEPENDS_remove = "dbus"

# Pull in patches
SRC_URI += "file://0001-wpa_ctrl_groups.patch \
            file://0002-Don-t-write-blob-back-when-saving-configuration.patch \
            file://0003-allow-wpa-supplicant-to-skip-date-check.patch \
            file://0004-Add-a-new-mixed-autoscan-module-for-better-targeted-scans.patch \
            file://0005-Add-flag-to-read-blob-only.patch \
            file://0007-Don-t-commit-the-temporary-config-if-there-was-an-er.patch \
            "
