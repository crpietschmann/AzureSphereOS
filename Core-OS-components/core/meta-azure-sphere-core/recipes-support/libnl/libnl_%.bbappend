#
# Customize libnl to exclude CLI tools
#

# disable CLI tools and debug
EXTRA_OECONF_append = " --disable-cli --disable-debug"

# since CLI is disabled don't try to build this package
PACKAGES_remove = "libnl-cli"

# Support building for native / SDK targets
BBCLASSEXTEND = "native nativesdk"
