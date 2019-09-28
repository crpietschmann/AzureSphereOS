#!/bin/bash
#
# Build Azure Sphere Core OS components
#

# Get script folder location
if [ -n "$BASH_SOURCE" ]; then
	SCRIPT_PATH="${BASH_SOURCE[0]}"
else
	echo "Warning: unsupported shell"
    exit 1
fi

SELF_PATH=`readlink -f "${SCRIPT_PATH}"`
SCRIPT_ROOT=`dirname "${SELF_PATH}"`
POKY_ROOT=${SCRIPT_ROOT}/../../poky
TEMPLATE_DIR=${SCRIPT_ROOT}/../conf-templates/os

export BUILD_ROOT=`readlink -m "${SCRIPT_ROOT}/../../build/os"`
export CACHE_ROOT=`readlink -m "${SCRIPT_ROOT}/../../build/cache"`
export TMPDIR=${BUILD_ROOT}/out
export BB_ENV_EXTRAWHITE="TMPDIR CACHE_ROOT"

LOG_DIR=${BUILD_ROOT}

if [ ! -d "$BUILD_ROOT" ]; then
    mkdir -p $BUILD_ROOT
fi

TEMPLATECONF=${TEMPLATE_DIR} MACHINE=mt3620 source ${POKY_ROOT}/oe-init-build-env ${BUILD_ROOT} >>${LOG_DIR}/env-setup.log 2>&1

bitbake azure-sphere-core