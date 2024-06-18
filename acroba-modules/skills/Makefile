
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


##---------------------------------------------------------------------------------

.PHONY: all 
all: build

.PHONY: build
build:
	$(MAKE) -e _BUILD_TAG=${_BUILD_TAG}\
		build_skills_msgs build_skills_base build_skills build_skills_vg

.PHONY: push
push:
	$(MAKE) -e _BUILD_TAG=${_BUILD_TAG} \
		push_skills_msgs push_skills_base push_skills push_skills_vg 

.PHONY: release
release: 
	$(MAKE) -e _BUILD_TAG=${_BUILD_TAG} _RELEASE_TAG=${_RELEASE_TAG} \
		release_skills_msgs release_skills_base release_skills release_skills_vg 

.PHONY: pull
pull:
	$(MAKE) -e _BUILD_TAG=${_BUILD_TAG} \
		pull_skills_msgs pull_skills_base pull_skills pull_skills_vg

.PHONY: switch
switch: switch_skills_msgs switch_skills_base switch_skills switch_skills_vg


##---------------------------------------------------------------------------------
# Build instructions per package

.PHONY: build_skills_msgs
build_skills_msgs: _build_skills_msgs 

.PHONY: build_skills
build_skills: _build_skills

.PHONY: build_skills_vg
build_skills_vg: _build_skills_vg

.PHONY: build_skills_base
build_skills_base:
	docker build --tag ${APPLICATION_NAME}/skills_base:${_BUILD_TAG} -f skills_utils/Dockerfile .
	docker tag ${APPLICATION_NAME}/skills_base:${_BUILD_TAG} acroba/skills_base

.PHONY: force 
force: 

_build_%: force 
	docker build --tag ${APPLICATION_NAME}/$*:${_BUILD_TAG} -f $*/Dockerfile ./$*
	docker tag ${APPLICATION_NAME}/$*:${_BUILD_TAG} acroba/$*

##---------------------------------------------------------------------------------
# Push single package

push_%: force
	docker push ${APPLICATION_NAME}/$*:${_BUILD_TAG}

##---------------------------------------------------------------------------------
# Release single package

release_%: force
	docker tag  ${APPLICATION_NAME}/$*:${_BUILD_TAG} ${APPLICATION_NAME}/$*:${_RELEASE_TAG}
	docker push ${APPLICATION_NAME}/$*:${_RELEASE_TAG}

##---------------------------------------------------------------------------------
# Pulling single package

pull_%: force 
	docker pull ${APPLICATION_NAME}/$*:${_BUILD_TAG}
	docker tag ${APPLICATION_NAME}/$*:${_BUILD_TAG} acroba/$* 

##---------------------------------------------------------------------------------
# switching generic tag 

switch_%: force 
	docker tag ${APPLICATION_NAME}/$*:${_BUILD_TAG} acroba/$* 
