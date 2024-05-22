FROM ros:noetic

RUN apt-get update && \
	apt install ros-$ROS_DISTRO-foxglove-bridge -y --no-install-recommends && \
	apt-get clean && rm -rf /var/lib/apt/lists/*
RUN pip install rosnumpy --no-cache-dir