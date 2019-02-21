# Lyo-based OSLC services

The services are generated using Lyo Designer 4.0.0-SNAPSHOT. Services rely on Lyo 4.0.0-SNAPSHOT libraries. Modelling project is located under 'model-sandbox' (for the services) and under `model-domain-pddl` (for the OSLC domains).

To run the services run:

    # from the project root
    make build && make restart-swarm

To initialise robots twinsc

    # from the project root
    cd lyo-services/scripts
    groovy init-twins.groovy multi 1 20

Logs can be retrieved using

    # when scaled to 1 instance
    docker service logs scott_sandbox-whc -f --no-task-ids  --raw

    # when scaled to more than 1 instance
    docker service logs scott_sandbox-twin -f --no-task-ids

General docker stats can be shown using

    watch 'docker service ls --format "table {{.Name}}\t{{.Replicas}}\t{{.Ports}}"'

and

    docker stats --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemPerc}}"

## Old README

Lyo-based services are built together, that's why they are all in the same subfolder.

In order to speed up the Docker build, the levels (don't confuse with Docker's _layers_) are organised this way:

- `scott-docker-base` layer
- `scott-sandbox-base` layer
- individual images with the webapps running under Jetty

The base image must be build only after the `m2_temp` folder has been initialised. In order to do this, run the following command:

   docker run -it --rm -v "$PWD":/src -v "$PWD/m2_temp":/root/.m2/repository -w /src maven:3-jdk-8 mvn clean package -f lyo-webapp-parent/pom.xml

The sandbox image builds the WAR packages with the webapps. To speed up the builds, layers are used extensively and the build process consists of two steps: fetching all Maven dependencies, and compiling the code. This way, a layer with the dependencies can be persisted across rebuilds.
