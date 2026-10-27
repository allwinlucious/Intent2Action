# One container with the Intrinsic layer. Build from the project root:
#   container/build_intrinsic.sh && container/stage_intrinsic.sh && docker build -f container/Dockerfile -t intrinsic-core:dev .
FROM us-central1-docker.pkg.dev/intrinsic-mirror/intrinsic-build-images/zenohd@sha256:9a8195c927169365e516309d55753b9c3a7484a59825cf4fbefb9125cfbdb01d AS zenoh

# Humble matches the ROS 2 distro Intrinsic's Bazel build links against.
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends python3-pip python3-yaml iproute2 ros-humble-rmw-cyclonedds-cpp && rm -rf /var/lib/apt/lists/* \
 && pip3 install --no-cache-dir grpcio==1.85.0 protobuf==7.36.2
# The version Isaac's bridge bundles, so both sides use the same definitions.
RUN git clone --depth 1 --branch 1.1.0 https://github.com/ros-simulation/simulation_interfaces /opt/ws/src/simulation_interfaces \
 && . /opt/ros/humble/setup.sh \
 && colcon build --base-paths /opt/ws/src --merge-install --install-base /opt/ros_ext

COPY --from=zenoh /zenohd /opt/zenoh/zenohd
COPY container/stage/bin /opt/intrinsic/bin
COPY container/stage/lib /opt/intrinsic/lib
COPY gen /opt/app/gen
COPY container/services /opt/app/container/services
COPY container/config /opt/app/container/config
COPY container/dds /opt/app/container/dds
COPY container/entrypoint.sh /opt/app/container/entrypoint.sh
COPY config /opt/app/config
COPY robot_model /opt/app/robot_model
COPY assets/robots /opt/app/assets/robots

ENV LD_LIBRARY_PATH=/opt/intrinsic/lib
ENV PYTHONPATH=/opt/app:/opt/app/gen
ENTRYPOINT ["/opt/app/container/entrypoint.sh"]
