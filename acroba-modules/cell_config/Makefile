REGISTRY_NAME ?= docker.irtjv.local
PROJECT_NAME=rosconfr24-ws4
APPLICATION_NAME ?= ${REGISTRY_NAME}/${PROJECT_NAME}
PACKAGE_NAME ?= cell-config-roscon

HASH ?= $(shell git log --format="%h" -n 1)

ifeq ($(TAG),)
	_BUILD_TAG ?= ${HASH}
else 
	_BUILD_TAG ?= ${TAG}
endif 

LABEL?=latest
_RELEASE_TAG ?= ${LABEL}

.PHONY: all
all: checkout build

.PHONY: checkout 
checkout:
	git submodule sync
	git submodule update --init --recursive

.PHONY: build
build:
	docker build -t ${APPLICATION_NAME}/${PACKAGE_NAME}:${_BUILD_TAG}  -f docker/Dockerfile .  
	docker tag ${APPLICATION_NAME}/${PACKAGE_NAME}:${_BUILD_TAG} acroba/${PACKAGE_NAME}	
	
.PHONY: push
push:
	docker push ${APPLICATION_NAME}/${PACKAGE_NAME}:${_BUILD_TAG}

.PHONY: release
release: 
	docker tag  ${APPLICATION_NAME}/${PACKAGE_NAME}:${_BUILD_TAG} ${APPLICATION_NAME}/${PACKAGE_NAME}:${_RELEASE_TAG}
	docker push ${APPLICATION_NAME}/${PACKAGE_NAME}:${_RELEASE_TAG}

.PHONY: pull
pull:
	docker pull ${APPLICATION_NAME}/${PACKAGE_NAME}:${_BUILD_TAG}
	docker tag ${APPLICATION_NAME}/${PACKAGE_NAME}:${_BUILD_TAG} acroba/${PACKAGE_NAME} 