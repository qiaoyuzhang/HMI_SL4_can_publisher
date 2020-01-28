#CMAKE_GENERATOR_NAME=Ninja
CMAKE_GENERATOR_NAME=Unix Makefiles
TIME_COMMAND=/usr/bin/time --format "buildtime: real=%e user=%U sys=%S [ %C ]"

STAGING_BUILD_TYPE=RelWithDebInfo
RELEASE_BUILD_TYPE=RelWithDebInfo
DEBUG_BUILD_TYPE=Debug

BUILD_DIR=build
INSTALL_DIR=opt

STAGING_DIR=staging
RELEASE_DIR=$(shell echo "${RELEASE_BUILD_TYPE}" | tr '[:upper:]' '[:lower:]')
DEBUG_DIR=$(shell echo "${DEBUG_BUILD_TYPE}" | tr '[:upper:]' '[:lower:]')

BUILD_COMMAND=$(MAKE) --jobs $(JOB_NO) --no-print-directory
BUILD_ID  ?= "latest"
BUILD_UID := $(shell id -u)
WORKSPACE ?= $(abspath $(shell pwd)/..)

# The list of files which should trigger a re-cmake. Captured via:
# find * -name CMakeLists.txt -or -name '*.cmake' | sort | sed -e '$ ! s/$/ \\/' -e '2,$ s/^/\t/'
CMAKE_DEPS=can_common/CMakeLists.txt \
           hmi_message/CMakeLists.txt

# If the .ONESHELL special target appears anywhere in the makefile then all recipe lines for each
# target will be provided to a single invocation of the shell.
.ONESHELL:

# Default to debug build
all: debug

.PHONY: hook_init
hook_init:
	@ git config core.hooksPath .githooks
	@ echo "May the force be with you!"

#################### RELEASE

${BUILD_DIR}/${RELEASE_DIR}/Makefile: $(CMAKE_DEPS)
	@ $(eval LOGFILE=$(shell echo cmake-`date +%Y%m%dT%H%M%S`.log))
	@ $(info build: configuring ${RELEASE_BUILD_TYPE} build via cmake [log: ${BUILD_DIR}/${RELEASE_DIR}/${LOGFILE}]...)
	@ mkdir -p ${BUILD_DIR}/${RELEASE_DIR}
	@ cd ${BUILD_DIR}/${RELEASE_DIR}
	@ $(TIME_COMMAND) cmake ../.. -DCMAKE_BUILD_TYPE="${RELEASE_BUILD_TYPE}" -DENABLE_DISTCC:BOOL=${ENABLE_DISTCC} -G"$(CMAKE_GENERATOR_NAME)" > ${LOGFILE} 2>&1 || (/usr/bin/tail -25 ${LOGFILE}; exit 1)

.PHONY: release
release: ${BUILD_DIR}/${RELEASE_DIR}/Makefile
	@ $(eval LOGFILE=$(shell echo build-`date +%Y%m%dT%H%M%S`.log))
	@ $(info build: building ${RELEASE_BUILD_TYPE} [log: ${BUILD_DIR}/${RELEASE_DIR}/${LOGFILE}]...)
	@ $(TIME_COMMAND) $(BUILD_COMMAND) -C "${BUILD_DIR}/${RELEASE_DIR}" 2>&1 | tee ${BUILD_DIR}/${RELEASE_DIR}/build-`date +%Y%m%dT%H%M%S`.log
	@ touch ${BUILD_DIR}/CATKIN_IGNORE
	@ cd ${BUILD_DIR}
	@ rm -f latest
	@ /bin/ln -s ${RELEASE_DIR} latest

#################### DEBUG

${BUILD_DIR}/${DEBUG_DIR}/Makefile: $(CMAKE_DEPS)
	@ $(eval LOGFILE=$(shell echo cmake-`date +%Y%m%dT%H%M%S`.log))
	@ $(info build: configuring ${DEBUG_BUILD_TYPE} build via cmake [log: ${BUILD_DIR}/${DEBUG_DIR}/${LOGFILE}]...)
	@ mkdir -p ${BUILD_DIR}/${DEBUG_DIR}
	@ cd ${BUILD_DIR}/${DEBUG_DIR}
	@ $(TIME_COMMAND) cmake ../.. -DCMAKE_BUILD_TYPE="${DEBUG_BUILD_TYPE}" -DENABLE_DISTCC:BOOL=${ENABLE_DISTCC} -G"$(CMAKE_GENERATOR_NAME)" > ${LOGFILE} 2>&1 || (/usr/bin/tail -25 ${LOGFILE}; exit 1)

.PHONY: debug
debug: ${BUILD_DIR}/${DEBUG_DIR}/Makefile
	@ $(eval LOGFILE=$(shell echo build-`date +%Y%m%dT%H%M%S`.log))
	@ $(info build: building ${DEBUG_BUILD_TYPE} [log: ${BUILD_DIR}/${DEBUG_DIR}/${LOGFILE}]...)
	@ $(TIME_COMMAND) $(BUILD_COMMAND) -C "${BUILD_DIR}/${DEBUG_DIR}" 2>&1 | tee ${BUILD_DIR}/${DEBUG_DIR}/build-`date +%Y%m%dT%H%M%S`.log
	@ touch ${BUILD_DIR}/CATKIN_IGNORE
	@ cd ${BUILD_DIR}
	@ rm -f latest
	@ /bin/ln -s "${DEBUG_DIR}" latest


#################### STAGING

${BUILD_DIR}/${STAGING_DIR}/Makefile: $(CMAKE_DEPS)
	@ $(eval LOGFILE=$(shell echo cmake-`date +%Y%m%dT%H%M%S`.log))
	@ $(info build: configuring STAGING (${STAGING_BUILD_TYPE}) build via cmake [log: ${BUILD_DIR}/${STAGING_DIR}/${LOGFILE}]...)
	@ mkdir -p ${BUILD_DIR}/${STAGING_DIR}
	@ cd ${BUILD_DIR}/${STAGING_DIR}
	@ $(TIME_COMMAND) cmake ../.. -DSTAGING:BOOL=ON -DCMAKE_BUILD_TYPE="${STAGING_BUILD_TYPE}" -DENABLE_DISTCC:BOOL=${ENABLE_DISTCC} -G"$(CMAKE_GENERATOR_NAME)" > ${LOGFILE} 2>&1 || (/usr/bin/tail -25 ${LOGFILE}; exit 1)

.PHONY: staging
staging: ${BUILD_DIR}/${STAGING_DIR}/Makefile
	@ $(eval LOGFILE=$(shell echo build-`date +%Y%m%dT%H%M%S`.log))
	@ $(info build: building STAGING (${STAGING_BUILD_TYPE}) [log: ${BUILD_DIR}/${STAGING_DIR}/${LOGFILE}]...)
	@ $(TIME_COMMAND) $(BUILD_COMMAND) -C "${BUILD_DIR}/${STAGING_DIR}" 2>&1 | tee ${BUILD_DIR}/${STAGING_DIR}/build-`date +%Y%m%dT%H%M%S`.log
	@ touch ${BUILD_DIR}/CATKIN_IGNORE
	@ cd ${BUILD_DIR}
	@ rm -f latest
	@ /bin/ln -s "${STAGING_DIR}" latest

####################

.PHONY: install
install:
	@ $(info build: installing latest build...)
	@ $(TIME_COMMAND) $(BUILD_COMMAND) -C "${BUILD_DIR}/latest" install
	@ touch ${INSTALL_DIR}/CATKIN_IGNORE

.PHONY: test
test:
	@ $(info build: testing latest build...)
	@ $(TIME_COMMAND) $(BUILD_COMMAND) -C "${BUILD_DIR}/latest" CTEST_OUTPUT_ON_FAILURE=1 check

.PHONY: package
package: staging
	@ $(info build: installing STAGING build...)
	@ $(TIME_COMMAND) $(BUILD_COMMAND) -C "${BUILD_DIR}/${STAGING_DIR}" install
	@ $(info build: packaging STAGING build...)
	@ $(TIME_COMMAND) $(BUILD_COMMAND) -C "${BUILD_DIR}/${STAGING_DIR}" package

.PHONY: clean
clean:
	@ $(TIME_COMMAND) rm -rf "${BUILD_DIR}" "${INSTALL_DIR}"
	@ $(info build: outputs cleaned.)

#.PHONY: package-with-docker compile-test-with-docker

#package-with-docker:
#	nvidia-docker run \
            --rm \
            -u $(BUILD_UID) \
            -e "HOME=$(WORKSPACE)" \
            -e "BUILD_NUMBER=$(BUILD_NUMBER)" \
            -e "WORKSPACE=$(WORKSPACE)" \
            -v $(WORKSPACE):$(WORKSPACE) \
            $(DOCKER_IMAGE) \
            $(WORKSPACE)/package.sh
