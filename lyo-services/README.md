# Sandbox Services based on Lyo

Lyo-based services are built together, that's why they are all in the same subfolder.

In order to speed up the Docker build, the levels (don't confuse with Docker's _layers_) are organised this way:

- `scott-docker-base` layer
- `scott-sandbox-base` layer
- individual images with the webapps running under Jetty

The base image must be build only after the `m2_temp` folder has been initialised. In order to do this, run the following command:

   docker run -it --rm -v "$PWD":/src -v "$PWD/m2_temp":/root/.m2/repository -w /src maven:3-jdk-8 mvn clean package -f lyo-webapp-parent/pom.xml

The sandbox image builds the WAR packages with the webapps. To speed up the builds, layers are used extensively and the build process consists of two steps: fetching all Maven dependencies, and compiling the code. This way, a layer with the dependencies can be persisted across rebuilds. 
