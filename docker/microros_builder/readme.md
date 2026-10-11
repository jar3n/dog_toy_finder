# How to Build micro-ROS

Instructions for using the micro-ROS builder docker container.

1. `cd` into the `docker/` folder at the repo root
2. Build the container (first time, or after Dockerfile changes):
   `docker compose build microros_builder`
3. Build the micro-ROS library:
   `docker compose run --rm microros_builder bash -ic "bash ./build-library.sh"`
4. Copy the output from `firmware/pico_microros/microros/builder/artefacts/`
   into `firmware/pico_microros/lib/libmicroros/`