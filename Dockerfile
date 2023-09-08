FROM ros:melodic-ros-core

ARG user=maciej

RUN useradd -ms /bin/bash ${user} && \
    echo "$user:$user" | chpasswd && \
    adduser ${user} sudo && \
    echo "$user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${user}
WORKDIR /home/${user}

# upgrade because melodic docker wasn't updated in quite long time
RUN sudo apt-get update && \
    sudo apt-get upgrade -y && \
    sudo apt-get install -y python-catkin-tools python3-setuptools python-pip python3-vcstool curl git python3-rosdep && \
    # todo: install it using rosdep
    pip install numpy && \
    mkdir -p catkin_ws/src && \
    sudo chown -R ${user}:${user} catkin_ws


WORKDIR /home/${user}/catkin_ws

RUN git clone -b melodic-devel https://github.com/ROBOTIS-GIT/open_manipulator.git src/open_manipulator && \
    git clone https://github.com/ROBOTIS-GIT/open_manipulator_controls.git src/open_manipulator_controls 

COPY --chown=${user}:${user} ./config/manipulator_cmd_vel_controller.py /home/$user/catkin_ws/src/open_manipulator_controls/open_manipulator_controllers/src

RUN sudo apt-get update && \
    sudo rosdep init && \
    rosdep update --include-eol-distros && \
    rosdep install --from-paths src --ignore-src -y && \
    catkin config --init --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build

RUN sudo apt-get update && \
    sudo apt-get install -y ros-melodic-joint-state-controller ros-melodic-joint-trajectory-controller

# Simulation
# RUN sudo apt-get update && \
#     sudo apt-get install -y ros-melodic-gazebo-ros ros-melodic-moveit-simple-controller-manager

WORKDIR /home/${user}

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/${user}/.bashrc && \
    echo "source /home/$user/catkin_ws/devel/setup.bash --extend" >> /home/${user}/.bashrc

COPY --chown=${user}:${user} ./ros_entrypoint.sh /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]