#!/bin/bash

docker container rm jupyterhub
docker run -p 8000:8000 --name jupyterhub -it kail4ek/jupyterhub:latest \
	jupyterhub --ip 0.0.0.0 --port 8000
