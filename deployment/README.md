# SCOTT sandbox using linked data

## Local setup

    make build && make restart-swarm

Navigate to http://localhost:8081/sandbox-twin/services/serviceProviders/robot_handcrafted/robots/1. You should see that the particular Robot resource has property *isAt: http://sandbox-warehouse:8080/sandbox-warehouse/services/serviceProviders/dummy/resources/waypoints/wp2*

> Note that Docker does not support external (from the host OS) name resoulution to the containers, so the waypoint link will be broken unless you update your `/etc/hosts` file.

## Deployment

    make build
    STACK=docker-compose.erdc.yml make restart-swarm
