# Ubuntu 20.04 with nvidia-docker2 beta opengl support
# todo(iche033) use the image from dockerhub when available
FROM osrf/mbzirc:mbzirc_sim_latest

# Copy entry point script, and set the entrypoint
COPY docker/cloudsim_sim/run_sim.bash ./
ENTRYPOINT ["./run_sim.bash"]
