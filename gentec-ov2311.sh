#!/bin/bash

# Modify this variable according to your own setup:
EVK_IP="192.168.101.135"

# The path to the specific driver we want to compile:
DRIVER_PATH=drivers/media/platform/mxc/capture/ov2311

export MAKEJOBS="-j $(nproc)"

# To set ARCH and CROSS_COMPILE variables:
source /opt/gentec-cli/3.1/environment-setup-aarch64-fslc-linux

KERNEL_VERSION="$(make kernelversion)$(scripts/setlocalversion)"

make M=${DRIVER_PATH}

scp ${DRIVER_PATH}/ov2311.ko root@${EVK_IP}:/lib/modules/${KERNEL_VERSION}/kernel/${DRIVER_PATH}/
