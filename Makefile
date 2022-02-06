.PHONY: build-garage-headless-ros run-garage-headless-ros \
	build-garage-nvidia-ros run-garage-nvidia-ros \
	build-nvidia-sawyer-sim run-nvidia-sawyer-sim \
	build-nvidia-sawyer-robot run-nvidia-sawyer-robot

# Path in host where the experiment data obtained in the container is stored
DATA_PATH ?= $(shell pwd)/data
# Set the environment variable MJKEY with the contents of the file specified by
# MJKEY_PATH.
MJKEY_PATH ?= ~/.mujoco/mjkey.txt
# Sets the add-host argument used to connect to the Sawyer ROS master
SAWYER_NET = "$(SAWYER_HOSTNAME):$(SAWYER_IP)"
ifneq (":", $(SAWYER_NET))
	ADD_HOST=--add-host=$(SAWYER_NET)
endif

build-nvidia-sawyer-robot: docker/docker-compose-nv-robot.yml docker/get_intera.sh
	docker-compose -f docker/docker-compose-nv-robot.yml build

run-nvidia-sawyer-robot: build-nvidia-sawyer-robot
ifeq (,$(ADD_HOST))
	$(error Set the environment variables SAWYER_HOST and SAWYER_IP)
endif
	xhost +local:docker
	docker run \
		--privileged \
		--init \
		-t \
		--rm \
		--runtime=nvidia \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v $(shell pwd)/docker/sawyer-robot/docker_code:/root/code \
		-e DISPLAY="${DISPLAY}" \
		-e QT_X11_NO_MITSHM=1 \
		--net="host" \
		$(ADD_HOST) \
		--name "sawyer-robot" \
		sawyer-noetic/nvidia-sawyer-robot bash
