# SCOTT sandbox

This repository contains all projects that are part of the *SCOTT Warehouse Sandbox* as described in the [SCOTT WP10](https://projects.avl.com/16/0094/WP10/default.aspx).

The repository consists of 10 projects that use Docker images individually and are orchestrated using Docker Compose (as of now). Below is a brief summary of each project.


## Warehouse Controller

Warehouse Controller (`/warehousecontroller`) is the entry point to the sandbox (currently).

Until the multi-objective optimisation (MOO) service is present, its implementation is [stubbed](https://en.wikipedia.org/wiki/Method_stub) via the [mission.json](warehousecontroller/mission.json) file.

Until the [Ontology server](#ontology-server) is up and serving the warehouse state, the controller fetches it from the [kb.json](warehousecontroller/kb.json) file.

> **NB!** The KB is not queried from the [Mock KB](#mock-kb) service!

Until the [Planner Reasoner](#planner-reasoner) is able to serve the PDDL problem file, it is generated in the `generatePddlProblemFile` method of the [mission2plan.py](warehousecontroller/mission2plan.py) file. Until the [Planner Reasoner](#planner-reasoner) is able to serve the PDDL domain file, it is fetched from the [whdomain-2.pddl](warehousecontroller/whdomain-2.pddl) file.

When the problem & domain PDDL files are ready, they are uploaded to the [Metric-FF Docker](#metric-ff-docker) service and *plan generation* is triggered.

> :point_right: Start by running the Decker Compose as described in the [Deployment](#deployment) section and running a [cURL test script](warehousecontroller/curltest3) to trigger plan generation. Also see [README](warehousecontroller/Readme).


## Deployment

Deployment project contains the Docker Compose [configuration](deployment/docker-compose.yml) that allows you to start all the elements of the sandbox demo:

* [Metric-FF](#metric-ff-docker)
* [Warehouse controller](#warehouse-controller)

> :point_right: Start by running `docker-compose up -d` command inside the `/deployment` subdirectory and reading its [README](deployment/README.md).


## Metric-FF Docker

Metric-FF Docker (`/ff-metric-docker`) exposes Metric-FF-based planning via a RESTful service running on port 5000.

> :point_right: Start by running the preconfigured [docker-compose environment](#deployment) and reading [project README](ff-metric-docker/README.md).


## Mock KB

Mock KB (`/mockkb`) exposes a mock knowledge base on port 5001.

> :point_right: Start by running the service using Docker ([README](mockkb/README.md)) and doing a GET request to `/kb/api/v1.0/waypoint`.


## Ontology server

Ontology server (`/ontology-server`) exposes named graphs in the [Cliopatria](http://cliopatria.swi-prolog.org/home) installation over HTTP in RDF format via an Nginx server.

> :point_right: Start by following the [Getting Started section](ontology-server/README.md#getting-started) in the README.


## Optic Docker

Optic Docker (`/optic-docker`) is a project that packages an alternative [OPTIC planner](https://nms.kcl.ac.uk/planning/software/optic.html) via Docker.

> **NB!** This project does not expose OPTIC via REST (yet)!

> :point_right: Start by reading the [README](optic-docker/README.md).


## OSLC Prolog

OSLC Prolog (`/oslc_prolog`) is a Prolog library designed to help developing OSLC-compliant REST services. Just ask Leo if you want to build one.


## PDDL Examples

PDDL Examples (`/pddl-examples`) �s a folder with example PDDL files (domain and problem files), just for learning and testing purposes of the planners.



## PDDL Planner

PDDL Planner (`/planner`) exposes PDDL planners via a REST API (so does the [Metric-FF Docker](#metric-ff-docker), but using dockerised Metric-FF). Theoretically supports both OPTIC and Metric-FF, but only implements support for Metric-FF. Requires local installation of Metric-FF (see `ff_path` in [plannerService.py](planner/plannerService.py)).

> :point_right: Start by using [Metric-FF Docker](#metric-ff-docker) and consult with Costa and Swarup if you need to touch this project.


## Planner Reasoner

Planner Reasoner (`/planner_reasoner`) uses an [internal planning ontology](planner_reasoner/rdf/base/pp.ttl) to transform the *warehouse state* (given in a form that adheres to the [warehouse ontology](planner_reasoner/rdf/base/warehouse_domain.ttl)) into the PDDL problem and domain files
.

> :point_right: Start by reading about the [`generate_pddl/2` predicate](planner_reasoner/lib/planner_reasoner.pl).

> **NB!** The project is called a *reasoner* because it is using Prolog reasoning facilities, it does not do any reasoning over the ontology and does not produce any inferred triples (as of now).
