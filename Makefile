# ----------------------------------------------------------------------
#  Robocup@Home ROS Noetic Docker Development
# ----------------------------------------------------------------------

#: Builds a Docker image with the corresponding Dockerfile file

# ----------------------------BUILD------------------------------------
# ---------Main----------
# No GPU
main.build:
	@./docker/scripts/build.bash --area=main

# CUDA 11.8 x86_64
main.build.cuda:
	@./docker/scripts/build.bash --area=main --use-cuda

# Jetson devices
main.build.jetson:
	@./docker/scripts/build.bash --area=main --jetson-l4t=35.4.1

# ----------------------------CREATE------------------------------------

main.create:
	@./docker/scripts/run.bash --area=main --volumes=$(volumes) --name=$(name)

main.create.cuda:
	@./docker/scripts/run.bash --area=main --use-cuda --volumes=$(volumes) --name=$(name)

main.create.jetson:
	@./docker/scripts/run.bash --area=main --jetson-l4t=35.4.1 --volumes=$(volumes) --name=$(name)

# ----------------------------START------------------------------------
# Start containers
main.up:
	@xhost +
	@docker start home-main

# ----------------------------STOP------------------------------------
# Stop containers
main.down:
	@docker stop home-main 

# ----------------------------RESTART------------------------------------
# Restart containers
main.restart:
	@docker restart home-main 

# ----------------------------LOGS------------------------------------
# Logs of the container
main.logs:
	@docker logs --tail 50 home-main

# ----------------------------SHELL------------------------------------
# Fires up a bash session inside the container
main.shell:
	@docker exec -it --user $(shell id -u):$(shell id -g) home-main

# ----------------------------REMOVE------------------------------------
# Remove container
main.remove:
	@docker container rm home-main

# ----------------------------------------------------------------------
#  General Docker Utilities

#: Show a list of images.
list-images:
	@docker image ls

#: Show a list of containers.
list-containers:
	@docker container ls -a