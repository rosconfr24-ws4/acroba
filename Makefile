REGISTRY_NAME ?= docker.irtjv.local
PROJECT_NAME=rosconfr24-ws4
APPLICATION_NAME ?= ${REGISTRY_NAME}/${PROJECT_NAME}

TAG=latest

CELL ?= cell-config-roscon
X11 ?= YES
GPU ?= YES
VG ?= YES

#----------------------------------------------------------------------------------

HASH ?= $(shell git log --format="%h" -n 1)

ifeq ($(TAG),)
	_BUILD_TAG ?= ${HASH}
else 
	_BUILD_TAG ?= ${TAG}
endif 

LABEL?=latest
_RELEASE_TAG ?= ${LABEL}

# docker container project name
PROJECT ?= acroba

_ABORT_ON_FAILURE ?= true

###################################################################################
# PLATFORM SETUP 
###################################################################################

.PHONY: all 
all: pull run

.PHONY: setup
setup: 
	./scripts/setup.sh

#-----------------------------------------------------------------------------------
# Docker images Management
#-----------------------------------------------------------------------------------

.PHONY: check-images
check_image = docker image inspect '--format={{join .RepoTags "\n"}}' acroba/$(1); echo "";

check-images:
	@if docker info >/dev/null 2>&1; then \
		$(call check_image,ros1-base-nvidia) \
		$(call check_image,acroba-base-ros1) \
		$(call check_image,acroba-base-ros1-gpu) \
		$(call check_image,skills_msgs) \
		$(call check_image,skills_base) \
		$(call check_image,skills) \
		$(call check_image,skills_vg) \
		$(call check_image,virtualgym_msgs) \
		$(call check_image,virtualgym) \
		$(call check_image,taskplanner) \
		$(call check_image,cell-config-base-gpu) \
		$(call check_image,cell-config-base) \
		$(call check_image,cell-config-roscon) \
	else \
		echo "Docker is not available. Skipping images checks."; \
	fi


define remove_images 
	ID=$$(docker image inspect --format '{{.Id}}' acroba/$(1) | cut -c8-19);\
	images=$$(docker images --filter=reference=*/rosconfr24-ws4/$(1) --format '{{.ID}}' | \
		awk '!seen[$$1]++' | awk -v MY_ID="$${ID}" '$$1 != MY_ID');\
	if [ -n "$$images" ]; then\
		echo "Removing images for $(1):\n $$images";\
	 	echo "$$images" | xargs docker rmi -f;\
	else\
		echo "No image to remove for acroba/$(1)";\
	fi
endef

.PHONY: clean-images
clean-images:
# remove local images that are not used by the platform, 
# i.e. images that are not referred to by the generic image tags acroba/xyz 
	@$(call remove_images,acroba-base-ros1)
	@$(call remove_images,ros1-base-nvidia)
	@$(call remove_images,acroba-base-ros1-gpu)
	@$(call remove_images,skills_msgs)
	@$(call remove_images,skills_base)
	@$(call remove_images,skills)
	@$(call remove_images,skills_vg)
	@$(call remove_images,virtualgym_msgs)
	@$(call remove_images,virtualgym)
	@$(call remove_images,taskplanner)
	@$(call remove_images,cell-config-base-gpu)
	@$(call remove_images,cell-config-base)
	docker image prune -f 


##----------------------------------------------------------------------------------
#  BUILDING  
##----------------------------------------------------------------------------------

.PHONY: build 
build: build_base-images build_cell-config-base modules_build

# instead of using only generic build_%, putting explicit rules to enable autocompletion 

.PHONY: build_base-images
build_base-images: build_ros1-base-nvidia build_acroba-base

.PHONY: build_ros1-base-nvidia
build_ros1-base-nvidia: _build_ros1-base-nvidia

# acroba base images --------------------------------------------------------------------

.PHONY: build_acroba-base
build_acroba-base: build_acroba-base-ros1.cpu build_acroba-base-ros1.gpu 

.PHONY: build_acroba-base-ros1.cpu
build_acroba-base-ros1.cpu:
	$(MAKE) -e _BUILD_TAG=${_BUILD_TAG} BUILD_ARG=${BUILD_ARG}\
		_build_acroba-base-ros1 \
			DOCKER_FOLDER="acroba-base"\
			BUILD_ARG="--build-arg BASE_IMAGE=ros1"

.PHONY: build_acroba-base-ros1.gpu
build_acroba-base-ros1.gpu:
	$(MAKE) -e _BUILD_TAG=${_BUILD_TAG} BUILD_ARG=${BUILD_ARG}\
		_build_acroba-base-ros1-gpu DOCKER_FOLDER="acroba-base"

.PHONY: build_acroba-base-ros2
build_acroba-base-ros2:
	$(MAKE) -e _BUILD_TAG=${_BUILD_TAG} BUILD_ARG=${BUILD_ARG}\
		_build_acroba-base-ros2\
			DOCKER_FOLDER="acroba-base"\
			BUILD_ARG="--build-arg BASE_IMAGE=ros2 --build-arg ROSx=ros2 \
			--build-arg INSTALL_DIR=install --build-arg SETUP_SCRIPT=local_setup.bash"


# cell config base --------------------------------------------------------------------

.PHONY: build_cell-config-base 
build_cell-config-base: build_cell-config-base-cpu build_cell-config-base-gpu

.PHONY: build_cell-config-base-cpu
build_cell-config-base-cpu:
	$(MAKE) -e _BUILD_TAG=${_BUILD_TAG} BUILD_ARG=${BUILD_ARG}\
		_build_cell-config-base BUILD_ARG="--build-arg BASE_IMAGE=acroba/skills_msgs"

.PHONY: build_cell-config-base-gpu
build_cell-config-base-gpu:
	$(MAKE) -e _BUILD_TAG=${_BUILD_TAG} BUILD_ARG=${BUILD_ARG}\
		_build_cell-config-base-gpu DOCKER_FOLDER="cell-config-base"


##--------------------------------------------------------------------------------------
#  RUNNING  
##--------------------------------------------------------------------------------------

.PHONY: ssh-config
ssh-config: 
	./scripts/setup_ssh_config.sh

.PHONY: run-config 
run-config:
	./scripts/setup_docker_config.sh --CELL=${CELL} --X11=${X11} --GPU=${GPU} --VG=${VG} --PROJECT=${PROJECT} --PROFILE=${PROFILE}

.PHONY: run-setup
run-setup: 
	@rm -rf shared/.setup
	@mkdir -p shared/.setup/startup
	@xhost +SI:localuser:root 
	
.PHONY: run
run:
	@$(MAKE) -s -e GPU=${GPU} X11=${X11} CELL=${CELL} VG=${VG} PROJECT=${PROJECT} PROFILE=acroba _run

.PHONY: _run
_run: ssh-config run-setup start-vg
	@$(MAKE) -s -e GPU=${GPU} X11=${X11} VG=${VG} CELL=${CELL} PROJECT=${PROJECT} PROFILE=${PROFILE} run-config | tee shared/.setup/startup/docker-compose.acroba.yml
	COMPOSE_IGNORE_ORPHANS=True COMPOSE_PROFILES=* docker compose -f shared/.setup/startup/docker-compose.acroba.yml up

.PHONY: run-dev
run-dev: stop-dev ssh-config run-setup start-vg
	@./scripts/setup_devcontainers.sh --CELL=${CELL} --X11=${X11} --GPU=${GPU} --VG=${VG} --dev
	@cp .devcontainer/docker-compose.acroba.yml shared/.setup/startup/docker-compose.acroba.yml
	@./scripts/run_devcontainers.sh

.PHONY: start-vg
start-vg:
ifeq ($(VG),WIN)
	@$(MAKE) -s -e start-win-vg
else ifeq ($(VG),WSL)
	@$(MAKE) -s -e start-wsl-vg
else
	@$(MAKE) -s -e start-vg_
endif 


.PHONY: start-win-vg
start-win-vg: 
	@echo "starting vg scene on windows..."
	@sed "s/ros_ip=localhost/ros_ip=$$(hostname -I)/" ./acroba-modules/VirtualGym/virtual_scenes/roscon_imr_cell_win/generic_cell_Data/config.ini.template > ./acroba-modules/VirtualGym/virtual_scenes/roscon_imr_cell_win/generic_cell_Data/config.ini
	@./acroba-modules/VirtualGym/virtual_scenes/roscon_imr_cell_win/generic_cell.exe &

.PHONY: start-wsl-vg
start-wsl-vg: 
	@echo "starting vg scene on wsl..."
	@sed "s/ros_ip=localhost/ros_ip=$$(hostname -I)/" ./acroba-modules/VirtualGym/virtual_scenes/roscon_imr_cell/roscon_imr_cell_Data/config.ini.template > ./acroba-modules/VirtualGym/virtual_scenes/roscon_imr_cell/roscon_imr_cell_Data/config.ini	
	@./acroba-modules/VirtualGym/virtual_scenes/roscon_imr_cell/roscon_imr_cell.x86_64 &

.PHONY: start-vg_
start-vg_: 
	@cat ./acroba-modules/VirtualGym/virtual_scenes/roscon_imr_cell/roscon_imr_cell_Data/config.ini.template > ./acroba-modules/VirtualGym/virtual_scenes/roscon_imr_cell/roscon_imr_cell_Data/config.ini	


.PHONY: stop-dev
stop-dev:
	@docker compose -p acroba-dev down --remove-orphans
	@xhost -
	@./scripts/clean_devcontainers_tmp.sh
	
.PHONY: stop
stop: 
	@docker compose -p acroba stop
	@xhost -  

.PHONY: down
down: 
	@docker compose -p acroba down
	@xhost -  


###################################################################################
# Platform version management
###################################################################################

.PHONY: push
push: _push modules_push

.PHONY: release
release: _release modules_release

.PHONY: pull
pull: _pull modules_pull

.PHONY: switch 
switch: switch_base-images modules_switch

##---------------------------------------------------------------------------------

.PHONY: _push
_push: 
	$(MAKE) -e _BUILD_TAG=${_BUILD_TAG} push_base-images

.PHONY: _release
_release: 
	$(MAKE) -e _BUILD_TAG=${_BUILD_TAG} _RELEASE_TAG=${_RELEASE_TAG} release_base-images
		
.PHONY: _pull
_pull: 
	$(MAKE) -e _BUILD_TAG=${_BUILD_TAG} pull_base-images

.PHONY: _switch
_switch: switch_base-images

# instead of generic push_%, pull_%, release_%, putting explicit rules to have autocompletion 

#---------------------
# PUSHING
#---------------------

.PHONY: push_base-images
push_base-images: push_ros1-base-nvidia push_acroba-base push_cell-config-base

.PHONY: push_ros1-base-nvidia
push_ros1-base-nvidia: _push_ros1-base-nvidia

# acroba base --------------------------------------------------------------------

.PHONY: push_acroba-base
push_acroba-base: push_acroba-base-ros1 push_acroba-base-ros1-gpu

.PHONY: push_acroba-base-ros1
push_acroba-base-ros1: _push_acroba-base-ros1 

.PHONY: push_acroba-base-ros1-gpu
push_acroba-base-ros1-gpu: _push_acroba-base-ros1-gpu
	
.PHONY: push_acroba-base-ros2
push_acroba-base-ros2: _push_acroba-base-ros2

# cell config base --------------------------------------------------------------------

.PHONY: push_cell-config-base 
push_cell-config-base: push_cell-config-base-cpu push_cell-config-base-gpu

.PHONY: push_cell-config-base-cpu
push_cell-config-base-cpu: _push_cell-config-base

.PHONY: push_cell-config-base-gpu
push_cell-config-base-gpu: _push_cell-config-base-gpu


#---------------------
# RELEASING
#---------------------

.PHONY: release_base-images
release_base-images: release_ros1-base-nvidia release_acroba-base release_cell-config-base

.PHONY: release_ros1-base-nvidia
release_ros1-base-nvidia: _release_ros1-base-nvidia

# acroba base --------------------------------------------------------------------

.PHONY: release_acroba-base
release_acroba-base: release_acroba-base-ros1 release_acroba-base-ros1-gpu

.PHONY: release_acroba-base-ros1
release_acroba-base-ros1: _release_acroba-base-ros1 

.PHONY: release_acroba-base-ros1-gpu
release_acroba-base-ros1-gpu: _release_acroba-base-ros1-gpu
	
.PHONY: release_acroba-base-ros2
release_acroba-base-ros2: _release_acroba-base-ros2

# cell config base --------------------------------------------------------------------

.PHONY: release_cell-config-base 
release_cell-config-base: release_cell-config-base-cpu release_cell-config-base-gpu

.PHONY: release_cell-config-base-cpu
release_cell-config-base-cpu: _release_cell-config-base

.PHONY: release_cell-config-base-gpu
release_cell-config-base-gpu: _release_cell-config-base-gpu


#---------------------
# SWITCHING versions
#---------------------

.PHONY: switch_base-images
switch_base-images: switch_ros1-base-nvidia switch_acroba-base switch_cell-config-base

.PHONY: switch_ros1-base-nvidia
switch_ros1-base-nvidia: _switch_ros1-base-nvidia

# acroba base --------------------------------------------------------------------

.PHONY: switch_acroba-base
switch_acroba-base: switch_acroba-base-ros1 switch_acroba-base-ros1-gpu

.PHONY: switch_acroba-base-ros1
switch_acroba-base-ros1: _switch_acroba-base-ros1 

.PHONY: switch_acroba-base-ros1-gpu
switch_acroba-base-ros1-gpu: _switch_acroba-base-ros1-gpu
	
.PHONY: switch_acroba-base-ros2
switch_acroba-base-ros2: _switch_acroba-base-ros2

# cell config base --------------------------------------------------------------------

.PHONY: switch_cell-config-base 
switch_cell-config-base: switch_cell-config-base-cpu switch_cell-config-base-gpu

.PHONY: switch_cell-config-base-cpu
switch_cell-config-base-cpu: _switch_cell-config-base

.PHONY: switch_cell-config-base-gpu
switch_cell-config-base-gpu: _switch_cell-config-base-gpu


#---------------------
# PULLING
#---------------------

.PHONY: pull_base-images
pull_base-images: pull_ros1-base-nvidia pull_acroba-base pull_cell-config-base

.PHONY: pull_ros1-base-nvidia
pull_ros1-base-nvidia: _pull_ros1-base-nvidia

# acroba base --------------------------------------------------------------------

.PHONY: pull_acroba-base
pull_acroba-base: pull_acroba-base-ros1 pull_acroba-base-ros1-gpu

.PHONY: pull_acroba-base-ros1
pull_acroba-base-ros1: _pull_acroba-base-ros1 

.PHONY: pull_acroba-base-ros1-gpu
pull_acroba-base-ros1-gpu: _pull_acroba-base-ros1-gpu
	
.PHONY: pull_acroba-base-ros2
pull_acroba-base-ros2: _pull_acroba-base-ros2

# cell config base --------------------------------------------------------------------

.PHONY: pull_cell-config-base 
pull_cell-config-base: pull_cell-config-base-cpu pull_cell-config-base-gpu

.PHONY: pull_cell-config-base-cpu
pull_cell-config-base-cpu: _pull_cell-config-base

.PHONY: pull_cell-config-base-gpu
pull_cell-config-base-gpu: _pull_cell-config-base-gpu


##---------------------------------------------------------------------------------
# Platform modules management 
##---------------------------------------------------------------------------------

MODULES := \
	acroba-modules/VirtualGym \
	acroba-modules/skills \
	acroba-modules/taskplanner \
	acroba-modules/cell_config \

.PHONY: modules_build modules_push modules_release modules_pull modules_switch

modules_build: _modules_build

modules_release:
	$(MAKE) -e TAG=${TAG} LABEL=${LABEL} _ABORT_ON_FAILURE=false _modules_release

modules_push:
	$(MAKE) -e TAG=${TAG} LABEL=${LABEL} _ABORT_ON_FAILURE=false _modules_push

modules_pull:
	$(MAKE) -e TAG=${TAG} LABEL=${LABEL} _ABORT_ON_FAILURE=false _modules_pull

modules_switch:
	$(MAKE) -e TAG=${TAG} LABEL=${LABEL} _ABORT_ON_FAILURE=false _modules_switch

    
.PHONY: force
force: 

_modules_%: force
	$(eval ERRORS :=)
	
	$(eval MAKE_CMD := ${MAKE} -e LABEL=${LABEL})
	@$(if $(TAG),\
		$(eval MAKE_CMD := ${MAKE_CMD} TAG=${TAG} $*),\
		$(eval MAKE_CMD := ${MAKE_CMD} $*)\
	)
	
	@for module in $(MODULES); do \
		if [ "${_ABORT_ON_FAILURE}" = "true" ]; then \
			(cd $$module && eval $(MAKE_CMD)); \
		else \
			(cd $$module && eval $(MAKE_CMD)) || ERRORS="$$module $$ERRORS"; \
		fi; \
	done; \
	if [ -n "$$ERRORS" ]; then \
        echo "Errors occurred in the modules: $$ERRORS"; \
        exit 1; \
    fi
	

#------------------------------------------------------------------------
# Generic Rules
#------------------------------------------------------------------------

# _build_<image_name> [DOCKER_FOLDER=<folder>] [BUILD_ARG="--build-arg <ARGS>"]
_build_%: force
	@$(if $(DOCKER_FOLDER),,\
		$(eval DOCKER_FOLDER := $*) \
	)
	docker build ${BUILD_ARG} -t ${APPLICATION_NAME}/$*:${_BUILD_TAG} -f docker/$(DOCKER_FOLDER)/Dockerfile .
	docker tag ${APPLICATION_NAME}/$*:${_BUILD_TAG} acroba/$*
	$(eval DOCKER_FOLDER :=) # unset the variable. 

_push_%: force
	docker push ${APPLICATION_NAME}/$*:${_BUILD_TAG}

_release_%: force
	docker tag  ${APPLICATION_NAME}/$*:${_BUILD_TAG} ${APPLICATION_NAME}/$*:${_RELEASE_TAG}
	docker push ${APPLICATION_NAME}/$*:${_RELEASE_TAG}

_pull_%: force
	docker pull ${APPLICATION_NAME}/$*:${_BUILD_TAG}
	docker tag ${APPLICATION_NAME}/$*:${_BUILD_TAG} acroba/$* 

_switch_%: force 
	docker tag ${APPLICATION_NAME}/$*:${_BUILD_TAG} acroba/$* 


