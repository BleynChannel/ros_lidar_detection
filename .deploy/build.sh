#!/usr/bin/env bash

BASEDIR = "$(dirname "$(readlink -f "$0")")"
if [ "$1" == "--rebuild" ]; then rm -rf "$BASEDIR/build"; fi
mkdir -p $BASEDIR/build

cp $BASEDIR/docker-compose.yml $BASEDIR/build

docker build -t bleyn/3d_detection:latest $BASEDIR/..
if [ ! -e "$BASEDIR/build/3d_detection.tar.gz" ]; then
	docker image save bleyn/3d_detection:latest | gzip > $BASEDIR/build/3d_detection.tar.gz
fi

echo "!/usr/bin/env bash
BASEDIR = \"$(dirname \"\$(readlink -f \"\$0\")\")\"

docker load < \$BASEDIR/3d_detection.tar.gz
docker-compose -f \$BASEDIR/build/docker-compose.yml up -d" > $BASEDIR/build/deploy.sh