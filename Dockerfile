FROM osrf/ros:humble-desktop-full

RUN apt-get update && \
    apt-get install -y \
        libasio-dev \
        libeigen3-dev \
        python3-pip \
        libxcb-* && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./3rd/TEASER-plusplus /tmp/TEASER-plusplus
RUN cd /tmp/TEASER-plusplus && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    cmake --build . --config Release -j$(nproc) && \
    cmake --install . && rm -rf /tmp/TEASER-plusplus

COPY ./3rd/gtsam /tmp/gtsam
RUN cd /tmp/gtsam && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    cmake --build . --config Release -j$(nproc) && \
    cmake --install . && rm -rf /tmp/gtsam

COPY ./3rd/Livox-SDK2 /tmp/Livox-SDK2
RUN cd /tmp/Livox-SDK2 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    cmake --build . --config Release -j$(nproc) && \
    cmake --install . && rm -rf /tmp/Livox-SDK2

ARG ONNX_VERSION="1.28.0"
RUN curl -L -o /tmp/onnxruntime.tgz \
    "https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/onnxruntime-linux-x64-${ONNX_VERSION}.tgz" && \
    mkdir -p /tmp/onnxruntime && \
    tar -xzf /tmp/onnxruntime.tgz -C /tmp/onnxruntime --strip-components=1 && \
    mkdir /usr/local/include/onnxruntime && \
    cp -r /tmp/onnxruntime/include/* /usr/local/include/onnxruntime && \
    mkdir /usr/local/lib64 && \
    cp -r /tmp/onnxruntime/lib/* /usr/local/lib64 && \
    cp -r /tmp/onnxruntime/lib/* /usr/local/lib && \
    rm -rf /tmp/onnxruntime /tmp/onnxruntime.tgz

ENV LD_LIBRARY_PATH="/usr/local/lib:/usr/local/lib64:${LD_LIBRARY_PATH}"

RUN pip3 install pyside6

WORKDIR /app
COPY ./src ./src
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build && rm -rf build/ log/"
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run hand_input_path hand_input_path_node"]
