FROM ros:noetic

# create workspace
RUN mkdir /shaobing
WORKDIR /root/shaobing

# TODO: clone projects
# RUN git clone .. 

# install dependencies and some tools
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y 
    # apt-get install ros-humble-foxglove-bridge wget htop vim -y && \
    # rm -rf /var/lib/apt/lists/*

# build
RUN . /opt/ros/noetic/setup.sh && catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release

# setup .zshrc
RUN echo 'export TERM=xterm-256color\n\
    source /shaobing/devel/setup.zsh\n' \
    >> /root/.zshrc

