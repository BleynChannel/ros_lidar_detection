FROM bleyn/mmdet3d-ros:1.0.0rc4-noetic

RUN pip install --no-cache-dir rosnumpy

COPY /app /app
WORKDIR /app

CMD [ "python", "main.py", "detection", "configs/config.py", "checkpoints/weights.pth" ]