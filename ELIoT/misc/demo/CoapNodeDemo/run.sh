#!/bin/bash
docker build -t debug .
docker run -it --link mi debug Rot.js mi -t