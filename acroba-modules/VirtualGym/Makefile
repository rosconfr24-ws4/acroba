REGISTRY_NAME ?= ghcr.io
PROJECT_NAME=rosconfr24-ws4
APPLICATION_NAME ?= ${REGISTRY_NAME}/${PROJECT_NAME}

HASH = $(shell git log --format="%h" -n 1)

ifeq ($(TAG),)
	_BUILD_TAG ?= ${HASH}
else 
	_BUILD_TAG ?= ${TAG}
endif 

LABEL?=latest
_RELEASE_TAG ?= ${LABEL}


.PHONY: all
all: build

.PHONY: build
build: build_vg_msgs build_vg 
	
.PHONY: push
push: push_vg_msgs push_vg 

.PHONY: release
release: release_vg_msgs release_vg 

.PHONY: pull
pull: pull_vg_msgs pull_vg

.PHONY: switch
switch: switch_vg_msgs switch_vg

.PHONY: bare_build
bare_build: pull_vg_msg_base pull_vg_base build_vg_msgs build_vg 

##---------------------------------------------------------------------------------
# Instructions per package
##---------------------------------------------------------------------------------

.PHONY: build_vg_msgs
build_vg_msgs:
	docker build -t ${APPLICATION_NAME}/virtualgym_msgs:${_BUILD_TAG} -f ./docker/Dockerfile.msgs .
	docker tag ${APPLICATION_NAME}/virtualgym_msgs:${_BUILD_TAG} acroba/virtualgym_msgs

.PHONY: build_vg
build_vg:
	docker build --tag ${APPLICATION_NAME}/virtualgym:${_BUILD_TAG} -f ./docker/Dockerfile.vg .
	docker tag ${APPLICATION_NAME}/virtualgym:${_BUILD_TAG} acroba/virtualgym

##---------------------------------------------------------------------------------

.PHONY: push_vg_msgs
push_vg_msgs: _push_virtualgym_msgs

.PHONY: push_vg
push_vg: _push_virtualgym

##---------------------------------------------------------------------------------

.PHONY: release_vg_msgs
release_vg_msgs: _release_virtualgym_msgs

.PHONY: release_vg
release_vg: _release_virtualgym

##---------------------------------------------------------------------------------

.PHONY: pull_vg_msgs
pull_vg_msgs: _pull_virtualgym_msgs

.PHONY: pull_vg
pull_vg: _pull_virtualgym

.PHONY: pull_vg_base
pull_vg_base: _pull_acroba-base-ros1-gpu

.PHONY: pull_vg_msg_base
pull_vg_msg_base: _pull_acroba-base-ros1

##---------------------------------------------------------------------------------

.PHONY: switch_vg_msgs
switch_vg_msgs: _switch_virtualgym_msgs

.PHONY: switch_vg
switch_vg: _switch_virtualgym


##---------------------------------------------------------------------------------
# Generic Instructions
##---------------------------------------------------------------------------------

_push_%:
	docker push ${APPLICATION_NAME}/$*:${_BUILD_TAG}

##---------------------------------------------------------------------------------

_release_%:
	docker tag  ${APPLICATION_NAME}/$*:${_BUILD_TAG} ${APPLICATION_NAME}/$*:${_RELEASE_TAG}
	docker push ${APPLICATION_NAME}/$*:${_RELEASE_TAG}

##---------------------------------------------------------------------------------

_pull_%:
	docker pull ${APPLICATION_NAME}/$*:${_BUILD_TAG}
	docker tag ${APPLICATION_NAME}/$*:${_BUILD_TAG} acroba/$* 

##---------------------------------------------------------------------------------

_switch_%: 
	docker tag ${APPLICATION_NAME}/$*:${_BUILD_TAG} acroba/$* 

