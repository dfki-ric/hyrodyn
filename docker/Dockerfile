FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Core build tools + dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    ca-certificates \
    libeigen3-dev \
    libyaml-cpp-dev \
    libboost-all-dev \    
    libtinyxml-dev \
    libgtest-dev \
 && rm -rf /var/lib/apt/lists/*

# Build GoogleTest static libraries (Ubuntu ships sources only)
RUN cd /usr/src/gtest && \
    cmake . && \
    make -j && \
    cp lib/*.a /usr/lib
