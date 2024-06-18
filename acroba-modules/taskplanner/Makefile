REGISTRY_NAME ?= docker.irtjv.local
PROJECT_NAME=rosconfr24-ws4
APPLICATION_NAME ?= ${REGISTRY_NAME}/${PROJECT_NAME}

HASH ?= $(shell git log --format="%h" -n 1)

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
build:
	docker build --tag ${APPLICATION_NAME}/taskplanner:${_BUILD_TAG} -f docker/Dockerfile .
	docker tag ${APPLICATION_NAME}/taskplanner:${_BUILD_TAG} acroba/taskplanner
	
.PHONY: push
push: _push_taskplanner

.PHONY: release
release: _release_taskplanner

.PHONY: pull
pull: _pull_taskplanner

.PHONY: switch
switch: _switch_taskplanner


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

