#
# Build settings for GCC for the Azure Sphere SDK
#
# Changes some meta-mingw settings
#
EXTRA_OECONF_append_sdkmingw32 = " --disable-multilib --enable-default-pie"
EXTRA_OECONF_remove_sdkmingw32 = " --disable-lto"
CANADIANEXTRAOS=""
