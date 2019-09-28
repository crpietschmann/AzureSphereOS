Azure Sphere / MT3620 Linux Kernel build instructions
=====================================================

Prereqs
-------
Building the Linux kernel requires a compatible version of GCC.  For building targetting the MT3620 this means you must have the GNU ARM Embedded Toolchain installed.  Instructions to install this depend on the operating system of your choice but often the native package manager can do this.  For example on Ubuntu you can install this with the following command:

`apt-get install gcc-arm-none-eabi`

Any version of this tool chain 4.0 or higher will work for building the kernel.  Testing has been done with both versions 4.9.3 and 6.3.0.

Preparing the source
--------------------

1.) Download the file linux-azure-sphere.tar.gz to your PC.
2.) Create a target directory for the source (referred to as `src` going forward).
3.) Change your working directory to the source folder and extract the source using `tar xzvf <path to downloaded file>`

Building the kernel image
---------------------

1.) Prepare the config settings using the MT3620 config file:

`make ARCH=arm defconfig KBUILD_DEFCONFIG=mt3620_defconfig`

2.) Build the kernel execute in place (XIP) image:

`make ARCH=arm CROSS_COMPILE=arm-none-eabi- -j 4 xipImage`
