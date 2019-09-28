Overview
========
Build is done using the Yocto and Open Embedded toolchain.  For more details please see the public documentation here:

https://www.yoctoproject.org/docs/2.3/mega-manual/mega-manual.html

These steps have been tested on Ubuntu 16.04 x64.  Additional steps may be needed for other configurations.

Setup
=====
Before building you need to install a number of prerequisitve packages.  The following command will install all needed items:

$ sudo apt-get install gawk wget git-core diffstat unzip texinfo gcc-multilib \
     build-essential chrpath socat cpio python python3 python3-pip python3-pexpect \
     xz-utils debianutils iputils-ping libsdl1.2-dev xterm

These setup steps only need to be executed once and can be skipped for subsequent builds.

Building Core OS components
===========================
To build core OS components run the following script:

$ meta-azure-sphere-core/scripts/build-os.sh

This builds the core GPL components needed by the build and creates a tarball containing output binaries.  The resulting tarball is located at:

build/os/out/deploy/images/mt3620/azure-sphere-core-mt3620.tar.gz

Building SDK components
=======================
To build core SDK components run the following script:

$ meta-azure-sphere-core/scripts/build-sdk.sh

The resulting SDK tarball is produced at:

build/sdk/out/deploy/sdk/azure-sphere-core-musl-i686-azure-sphere-core-cortexa7t2hf-neon-toolchain-2.3.1.tar.xz

Packaging and installing a replacement GDBServer application
============================================

Packaging and installing a replacement GDBServer requires Visual Studio Tools Preview for Azure Sphere installed on a Windows 10 PC.  Follow the instructions provided with the tools to prepare your device for development.

To build the replacement GDBServer image package:

1.) Copy the /usr/gdbserver_files folder from the tarball of the OS build previously produced to the Windows 10 PC.  This should contain the files app_manifest.json and bin/gdbserver.
2.) On the PC open the Azure Sphere Developer Command Prompt
3.) From the command prompt package the app by running:

azsphere image package --input <path to gdbserver_files> --output gdbserver.imagepackage

4.) To load the new image onto a device run:

azsphere device sideload deploy --imagepackage gdbserver.imagepackage