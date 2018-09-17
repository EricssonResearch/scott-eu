#!/bin/bash
docker build -t server .
docker run --rm -ti --name mi server