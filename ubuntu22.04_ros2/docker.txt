FROM osrf/ros:humble-desktop-full
############################## SYSTEM PARAMETERS ##############################
# * Arguments
ARG USER=initial
ARG GROUP=initial
ARG UID=1000
ARG GID="${UID}"
ARG SHELL=/bin/bash
ARG HARDWARE=x86_64
ARG ENTRYPOINT_FILE=entrypint.sh

# * Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
# ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# * Setup users and groups
RUN groupadd --gid "${GID}" "${GROUP}" \
    && useradd --gid "${GID}" --uid "${UID}" -ms "${SHELL}" "${USER}" \
    && mkdir -p /etc/sudoers.d \
    && echo "${USER}:x:${UID}:${UID}:${USER},,,:/home/${USER}:${SHELL}" >> /etc/passwd \
    && echo "${USER}:x:${UID}:" >> /etc/group \
    && echo "${USER} ALL=(ALL) NOPASSWD: ALL" > "/etc/sudoers.d/${USER}" \
    && chmod 0440 "/etc/sudoers.d/${USER}"

# * Replace apt urls
# ? Change to tku
RUN sed -i 's@archive.ubuntu.com@ftp.tku.edu.tw@g' /etc/apt/sources.list
# ? Change to Taiwan
# RUN sed -i 's@archive.ubuntu.com@tw.archive.ubuntu.com@g' /etc/apt/sources.list

# * Time zone
ENV TZ=Asia/Taipei
RUN ln -snf /usr/share/zoneinfo/"${TZ}" /etc/localtime && echo "${TZ}" > /etc/timezone

# * Copy custom configuration
# ? Requires docker version >= 17.09
COPY --chmod=0775 ./${ENTRYPOINT_FILE} /entrypoint.sh
COPY --chown="${USER}":"${GROUP}" --chmod=0775 config config

############################### INSTALL #######################################
# * Install packages
RUN apt update \
    && apt install -y --no-install-recommends \
        sudo \
        git \
        htop \
        wget \
        curl \
        psmisc \
        # * Shell
        tmux \
        terminator \
        # * base tools
        udev \
        libtool \
        python3-pip \
        python3-dev \
        python3-setuptools \
        python3-colcon-common-extensions \
        software-properties-common \
        lsb-release \
        ros-humble-rmw-cyclonedds-cpp \
        # * Work tools
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# Install RealSense dependencies
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# 替换原有的 RealSense 安装部分
RUN apt update && apt install -y --no-install-recommends \
    librealsense2 \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
    ros-humble-librealsense2* \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-diagnostic-updater \
    ros-humble-moveit \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists

# Install PaddleOCR GPU dependencies
RUN apt update && apt install -y --no-install-recommends \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    # OMPL dependencies
    g++ \
    cmake \
    pkg-config \
    libboost-serialization-dev \
    libboost-filesystem-dev \
    libboost-system-dev \
    libboost-program-options-dev \
    libboost-test-dev \
    libeigen3-dev \
    libode-dev \
    libyaml-cpp-dev \
    # Additional dependencies
    castxml \
    libboost-python-dev \
    libboost-numpy-dev \
    python3-numpy \
    pypy3 \
    python3-pyqt5.qtopengl \
    freeglut3-dev \
    libassimp-dev \
    python3-opengl \
    python3-flask \
    python3-celery \
    libccd-dev \
    libfcl-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists 

# Install Python packages for PaddleOCR GPU
RUN pip3 install --upgrade pip && \
    pip3 install paddlepaddle-gpu==2.4.2.post117 -f https://www.paddlepaddle.org.cn/whl/linux/mkl/avx/stable.html && \
    pip3 install "paddleocr>=2.0.1" && \
    pip3 install -vU https://github.com/CastXML/pygccxml/archive/develop.zip pyplusplus && \
    pip3 install -vU PyOpenGL-accelerate

# Install additional ROS packages
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-geometric-shapes \
    ros-humble-srdfdom \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists 

############################## USER CONFIG ####################################
# * Switch user to ${USER}
USER ${USER}

RUN ./config/shell/bash_setup.sh "${USER}" "${GROUP}" \
    && ./config/shell/terminator/terminator_setup.sh "${USER}" "${GROUP}" \
    && ./config/shell/tmux/tmux_setup.sh "${USER}" "${GROUP}" \
    && sudo rm -rf /config

# Set environment variables
ENV CXX=g++
ENV MAKEFLAGS="-j $(nproc)"
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib64:\$LD_LIBRARY_PATH" >> ~/.bashrc && \
    echo "export PATH=/usr/local/cuda/bin:\$PATH" >> ~/.bashrc

# * Switch workspace to ~/work
RUN sudo mkdir -p /home/"${USER}"/work
WORKDIR /home/"${USER}"/work

# * Make SSH available
EXPOSE 22

ENTRYPOINT [ "/entrypoint.sh", "terminator" ]