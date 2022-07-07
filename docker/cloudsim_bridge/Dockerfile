# Ubuntu 20.04 with nvidia-docker2 beta opengl support
FROM osrf/mbzirc:mbzirc_sim_latest

# Copy entry point script, and set the entrypoint
COPY docker/cloudsim_bridge/run_bridge.bash ./
ENTRYPOINT ["./run_bridge.bash"]
