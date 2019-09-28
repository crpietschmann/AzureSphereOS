# Base image containing core components

# Derive from core-image-minimal
require recipes-core/images/core-image-minimal.bb

# Output a tarball
IMAGE_FSTYPES = "tar.gz"

# Add libnl
IMAGE_INSTALL_append = " libnl libnl-genl "

# Add WPA Supplicant
IMAGE_INSTALL_append = " wpa-supplicant "

# Add GDB Server
IMAGE_INSTALL_append = " gdb "

# Build a simple standalone SDK
SDK_PACKAGING_COMMAND = ""