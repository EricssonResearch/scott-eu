# SCOTT sandbox using linked data

## Getting started

Build all WAR packages and set the Docker Compose up:

    make

Navigate to http://localhost:8081/sandbox-twin/services/serviceProviders/robot_handcrafted/robots/1. You should see that the particular Robot resource has property *isAt: http://sandbox-warehouse:8080/sandbox-warehouse/services/serviceProviders/dummy/resources/waypoints/wp2*

> Note that Docker does not support external (from the host OS) name resoulution to the containers, so the waypoint link will be broken unless you update your `/etc/hosts` file.
 
## Old instructions

In order to run the adaptor, go to the `sandbox-warehouse-webapp` folder and run:

    mvn clean jetty:run-exploded

If you have Docker, you may run the following commands instead:

    docker build -t sandbox-warehouse .
    docker run -p 8080:8080 sandbox-warehouse

Navigate to http://localhost:8080/sandbox-warehouse/
