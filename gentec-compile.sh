#!/bin/bash

export MAKEJOBS="-j $(nproc)"

# To set ARCH and CROSS_COMPILE variables:
source /opt/gentec-cli/3.1/environment-setup-aarch64-fslc-linux

make
make modules_prepare
make modules
