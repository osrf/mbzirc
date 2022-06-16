# Ubuntu 20.04 with nvidia-docker2 beta opengl support
# todo(iche033) use the image from dockerhub when available
# FROM osrf/mbzirc:mbzirc_sim_latest
FROM mbzirc_sim

# Copy entry point script, and set the entrypoint
COPY docker/cloudsim_bridge/run_bridge.bash ./
ENTRYPOINT ["./run_bridge.bash"]
