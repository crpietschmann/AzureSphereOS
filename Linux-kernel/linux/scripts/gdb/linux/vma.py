# SPDX-License-Identifier: GPL-2.0
#
# gdb helper commands and functions for Linux kernel debugging
#
#  VMA dumpers
#
# Copyright (c) Microsoft Corp, 2019
#
# This work is licensed under the terms of the GNU GPL version 2.
#

import gdb

from linux import utils
from linux import cpus

vma_type = utils.CachedType("struct vm_area_struct")

def print_vma(vma):
    if vma.type.code in (gdb.TYPE_CODE_PTR, gdb.TYPE_CODE_INT):
        vma = vma.cast(vma_type.get_type().pointer())

    gdb.write("start:0x%-16x end:0x%-16x pgoff:%-8x prot:%-3x flags:%-8x\n" % (vma['vm_start'],
                                                                               vma['vm_end'],
                                                                               vma['vm_pgoff'],
                                                                               vma['vm_page_prot'],
                                                                               vma['vm_flags']))
    fil = vma['vm_file']
    if fil != 0:
        gdb.write("\tpath: %s\n" % (utils.dentry_name(fil['f_path']['dentry'])))


class LxVma(gdb.Command):
    """Print the textual representation of a vm_area_struct"""

    def __init__(self):
        super(LxVma, self).__init__("lx-vma", gdb.COMMAND_DATA,
                                    gdb.COMPLETE_EXPRESSION)

    def invoke(self, args, from_tty):
        print_vma(gdb.parse_and_eval(args))


LxVma()

mm_struct = utils.CachedType("struct mm_struct")

class LxVmas(gdb.Command):
    """Print all vm_area_structs in a given mm

lx-vmas [MM-STRUCT]: Lists information about every VMA in a given process memory
map. If the MM-STRUCT pointer is omitted, this prints the current task's
active_mm."""

    def __init__(self):
        super(LxVmas, self).__init__("lx-vmas", gdb.COMMAND_DATA,
                                     gdb.COMPLETE_EXPRESSION)

    def invoke(self, args, from_tty):
        def _get_vmas(cur_vma):
            while cur_vma:
                yield cur_vma
                cur_vma = cur_vma['vm_next']

        argv = gdb.string_to_argv(args)
        if len(argv) >= 1:
            mm = gdb.parse_and_eval(args)
        else:
            mm = cpus.current_task()['active_mm']

        for vma in _get_vmas(mm['mmap']):
            gdb.write("0x%x: " % (vma))
            print_vma(vma)

LxVmas()
