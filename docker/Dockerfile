FROM ros:melodic-ros-base

WORKDIR /workspace  

# Setting environemnt variables.
ENV LANG=C.UTF-8

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository ppa:deadsnakes/ppa

# Build tools and other requirements
RUN apt-get update && \
    apt-get install -y curl \
    git \
    vim \
    wget \
    build-essential

# Install Python, and other requirements
RUN apt-get update &&  \
    apt-get install -y python \
    python-dev \
    python-pip 


RUN git clone https://github.com/Gemini-team/ros_adapter.git

