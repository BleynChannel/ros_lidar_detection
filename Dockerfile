FROM bleyn/mmdet3d-ros:1.0.0rc4-noetic

COPY /app /app
WORKDIR /app

CMD [ "python", "main.py" ]