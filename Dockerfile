FROM ros:noetic-ros-base

# modify source.list
RUN cp /etc/apt/sources.list /etc/apt/sources.list.bak && rm /etc/apt/sources.list && \
    echo "deb http://mirrors.aliyun.com/ubuntu/ focal main restricted universe multiverse \n \
          deb http://mirrors.aliyun.com/ubuntu/ focal-security main restricted universe multiverse\n \
          deb http://mirrors.aliyun.com/ubuntu/ focal-updates main restricted universe multiverse\n \
          deb http://mirrors.aliyun.com/ubuntu/ focal-proposed main restricted universe multiverse\n \
          deb http://mirrors.aliyun.com/ubuntu/ focal-backports main restricted universe multiverse\n \
          deb-src http://mirrors.aliyun.com/ubuntu/ focal main restricted universe multiverse\n \
          deb-src http://mirrors.aliyun.com/ubuntu/ focal-security main restricted universe multiverse\n \ 
          deb-src http://mirrors.aliyun.com/ubuntu/ focal-updates main restricted universe multiverse\n \
          deb-src http://mirrors.aliyun.com/ubuntu/ focal-proposed main restricted universe multiverse\n \
          deb-src http://mirrors.aliyun.com/ubuntu/ focal-backports main restricted universe multiverse" \
        >> /etc/apt/sources.list
RUN apt-get update && apt-get upgrade -y

# copy this project into dorcker image
RUN apt-get install -y git zsh vim
COPY . /shaobing/

# install dependencies and some tools
RUN apt-get install ros-noetic-eigen-conversions -y && \
    apt install clang-12 clangd-12 clang-format-12 -y
RUN  ln -s /usr/bin/clang-12 /usr/bin/clang && \
     ln -s /usr/bin/clang++-12 /usr/bin/clang++ && \
     ln -s /usr/bin/clangd-12 /usr/bin/clangd && \
     ln -s /usr/bin/clang-format-12 /usr/bin/clang-format
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# build
# RUN rosdep update
# WORKDIR /shaobing
# RUN rosdep install --from-paths src --ignore-src -r -y
# RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git /ws_livox/src/livox_ros_driver2
# RUN . /opt/ros/noetic/setup.sh
# RUN cd /ws_livox/src/livox_ros_driver2 && ./build.sh ROS1
# RUN . /opt/ros/noetic/setup.sh && catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release

# # setup .zshrc
# RUN echo 'export TERM=xterm-256color\n\
#     source /shaobing/devel/setup.zsh\n' \
#     >> /root/.zshrc

