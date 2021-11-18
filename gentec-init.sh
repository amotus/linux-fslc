#!/bin/bash

source /opt/gentec-cli/3.1/environment-setup-aarch64-fslc-linux

make defconfig KBUILD_DEFCONFIG=diamentis_defconfig
