CANADIANEXTRAOS=""

# Enable ncurses text UI for GDB clients
PACKAGECONFIG_append = " tui"
# Windows SDK version doesn't have curses/ncurses
PACKAGECONFIG_remove_sdkmingw32 = " tui"

# Cleanup all versions of gdb build (poky and azure-sphere-core)
do_install_append_sdkmingw32() {
    rm ${D}/${bindir}/${TARGET_PREFIX}gdb-add-index*
}