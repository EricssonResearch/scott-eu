# SCOTT sandbox using linked data

## Getting started

In order to run the adaptor, go to the `sandbox-warehouse-webapp` folder and run:

    mvn clean jetty:run-exploded

If you have Docker, you may run the following commands instead:

    docker build -t sandbox-warehouse .
    docker run -p 8080:8080 sandbox-warehouse

Navigate to http://localhost:8080/sandbox-warehouse/
