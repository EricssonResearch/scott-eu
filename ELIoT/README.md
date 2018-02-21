## Getting started

If you have `make`:

    make up

Otherwise:

    mvn -f sandbox-leshan-server/pom.xml clean package
    docker-compose build
    docker-compose up
