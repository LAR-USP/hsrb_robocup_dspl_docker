#!/bin/bash
DOCKER_GATEWAY=172.19.0.1
FLASK_ENV=development FLASK_APP=inference_server.py flask run --host=$DOCKER_GATEWAY
