FROM devrt/ros-devcontainer-vscode:melodic-desktop

USER root

# Most of the code is originated from:
# https://gitlab.com/nvidia/container-images/cuda/-/blob/master/dist/10.0/ubuntu18.04-x86_64/base/Dockerfile

RUN python -m pip install requests

USER developer

# CMD ["/bin/bash", "rviz", "-d", "$(rospack find hsrb_rosnav_config)/launch/hsrb.rviz"]
