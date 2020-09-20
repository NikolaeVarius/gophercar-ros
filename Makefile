ros-base-build:
	docker build -f docker/Dockerfile-ros-base . -t local/ros-base:latest

ros-joy-build:
	docker build -f docker/Dockerfile-ros-joy . -t local/ros-joy:latest