# syntax = docker/dockerfile:1.3

ARG FROM_IMAGE=ros:humble


# Multi-stage for caching
FROM $FROM_IMAGE AS cacher

# Clone source
WORKDIR /home/ubuntu/starter_repo
COPY . .

# Copy manifests for caching
RUN find . -mindepth 1 -maxdepth 2 -name "src" -type d -printf '%P\n' \
      | xargs -I % mkdir -p /tmp/starter_repo/% && \
    find . -name "package.xml" \
      | xargs cp --parents -t /tmp/starter_repo && \
    find . -name "COLCON_IGNORE" \
      | xargs cp --parents -t /tmp/starter_repo || true


# Multi-stage for building
FROM $FROM_IMAGE AS installer

# Config dependencies install
ENV DEBIAN_FRONTEND=noninteractive

# TODO cache apt installs: https://gist.github.com/dergachev/8441335
# gives the following error:
# E: Failed to fetch http://packages.ros.org/ros2/ubuntu/dists/focal/InRelease  403  Forbidden [IP: 172.17.0.1 8000]
# E: The repository 'http://packages.ros.org/ros2/ubuntu focal InRelease' is not signed.

# REMOVEME: packages need to be upgraded from kisak-mesa repo
# libegl-mesa0 libgbm1 libgl1-mesa-dev
# libgl1-mesa-dri libglapi-mesa libglx-mesa0 libllvm15
RUN \
    --mount=type=cache,target=/var/cache/apt,mode=0777,sharing=locked \
    --mount=type=cache,target=/root/.cache/pip,mode=0777,sharing=locked \
    apt update && apt install -y \
        software-properties-common && \
    add-apt-repository ppa:kisak/kisak-mesa && \
    apt update && apt upgrade -y \
        libegl-mesa0 \
        libgbm1 \
        libgl1-mesa-dev \
        libgl1-mesa-dri \
        libglapi-mesa \
        libglx-mesa0 \
        libllvm15

# Set root password
RUN echo "root:pass"|chpasswd

# Create new non-priviliged user
RUN useradd ubuntu --create-home --shell /bin/bash && \
    echo "ubuntu:pass"|chpasswd && \
    usermod -aG sudo ubuntu

# Clone gazebo models
RUN git clone https://github.com/osrf/gazebo_models.git && \
    mv gazebo_models/* /usr/share/gazebo-11/models || true

# Install system dependencies with caching apt and pip package lists
COPY apt-dependencies.txt apt-dependencies.txt
RUN rm -f /etc/apt/apt.conf.d/docker-clean; \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache
RUN \
    --mount=type=cache,target=/var/cache/apt,mode=0777,sharing=locked \
    --mount=type=cache,target=/root/.cache/pip,mode=0777,sharing=locked \
    apt update && \
    yes | unminimize && \
    xargs -a apt-dependencies.txt apt install -y --no-install-recommends && \
    pip3 install \
        git+https://github.com/ruffsl/colcon-cache.git \
        git+https://github.com/ruffsl/colcon-clean.git && \
    colcon mixin update && \
    colcon metadata update

# Install dependencies
WORKDIR /home/ubuntu/starter_repo/workspace
COPY --from=cacher /tmp/starter_repo/workspace/src ./src
RUN \
    --mount=type=cache,target=/var/cache/apt,mode=0777,sharing=locked \
    --mount=type=cache,target=/root/.ros/rosdep,mode=0777,sharing=locked \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt update && \
    rosdep update && \
    rosdep install -y --from-paths src --ignore-src --rosdistro=$ROS_DISTRO \
        --skip-keys slam_toolbox


# Install pip packages
COPY pip-dependencies.txt pip-dependencies.txt
RUN \
    --mount=type=cache,target=/root/.cache/pip,mode=0777,sharing=locked \
    pip3 install -r pip-dependencies.txt

# Source .bashrc even for non-interactive shells
RUN sed -e '/[ -z "$PS1" ] && return/s/^/#/g' -i /home/ubuntu/.bashrc
RUN echo "source /home/ubuntu/starter_repo/environment" >> /home/ubuntu/.bashrc

# Switch back to non-root user
USER ubuntu
