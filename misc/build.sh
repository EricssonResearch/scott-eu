#!/usr/bin/env bash

(cd ff-metric-docker && docker build -t scott/misc-ff-metric .)
(cd warehousecontroller && docker build -t scott/misc-warehousecontroller .)
docker-compose up -d
