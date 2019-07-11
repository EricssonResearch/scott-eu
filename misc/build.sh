#!/usr/bin/env bash

(cd ff-metric-docker && docker build -t scott/misc-ff-metric .)
(cd optic-docker && docker build -t scott/misc-optic .)
(cd warehousecontroller && docker build -t scott/misc-warehousecontroller .)
(cd emq && docker build -t scott/misc-eqmttd .)
# TODO cli-whc
# TODO ELIoT
