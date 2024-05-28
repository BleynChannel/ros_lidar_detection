ARG ROS_DISTRO=noetic

FROM bleyn/mmdet3d-ros:1.0.0rc4-$ROS_DISTRO

RUN pip install --no-cache-dir rosnumpy

COPY /app /app
WORKDIR /app

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc