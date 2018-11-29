# SCOTT sandbox

<a href="https://travis-ci.org/EricssonResearch/scott-eu"><img src="https://travis-ci.org/EricssonResearch/scott-eu.svg?branch=master"/></a>

This repository contains all projects that are part of the *SCOTT Warehouse Sandbox* as described in the [SCOTT WP10](https://projects.avl.com/16/0094/WP10/default.aspx). SCOTT (www.scottproject.eu)  has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No 737422.

The repository consists of 9 projects that use Docker images individually and are orchestrated using Docker Compose (as of now). Below is a brief summary of each project.


## Warehouse Controller

Warehouse Controller (`/warehousecontroller`) is the entry point to the sandbox (currently).

Until the multi-objective optimisation (MOO) service is present, its implementation is [stubbed](https://en.wikipedia.org/wiki/Method_stub) via the [mission.json](warehousecontroller/mission.json) file.

Until the [Ontology server](#ontology-server) is up and serving the warehouse state, the controller fetches it from the [kb.json](warehousecontroller/kb.json) file.

Until the [Planner Reasoner](#planner-reasoner) is able to serve the PDDL problem file, it is generated in the `generatePddlProblemFile` method of the [mission2plan.py](warehousecontroller/mission2plan.py) file. Until the [Planner Reasoner](#planner-reasoner) is able to serve the PDDL domain file, it is fetched from the [whdomain-2.pddl](warehousecontroller/whdomain-2.pddl) file.

When the problem & domain PDDL files are ready, they are uploaded to the [Metric-FF Docker](#metric-ff-docker) service and *plan generation* is triggered.

> :point_right: Start by running the Docker Compose as described in the [Deployment](#deployment) section and running a [cURL test script](warehousecontroller/curltest3) to trigger plan generation. Also see [README](warehousecontroller/Readme).


## Simulated Environment

This contains simulations required by the sandbox. It currently consists of the automated warehouse simulated in [VREP](http://www.coppeliarobotics.com/downloads.html) with all its elements and robots.
Until the multi-objective optimisation (MOO) service is present, its results should be provided by the [mission.json](warehousecontroller/mission.json) file.
Until it contains the discret event simulations for modeling the supply chain dynamics, all required information should be provided  by the [mission.json](warehousecontroller/mission.json) file.

Access to this scene (both reading and control) is currently implemented by using VREP's remoteAPI in python. This is expected to change once [ROS](http://www.ros.org/) is deployed and used for controlling the robots in the scene.

## Deployment

Deployment project contains the Docker Compose [configuration](deployment/docker-compose.yml) that allows you to start all the elements of the sandbox demo:

* [Metric-FF](#metric-ff-docker)
* [Warehouse controller](#warehouse-controller)

> :point_right: Start by running `docker-compose up -d` command inside the `/deployment` subdirectory and reading its [README](deployment/README.md).


## Metric-FF Docker

Metric-FF Docker (`/ff-metric-docker`) exposes Metric-FF-based planning via a RESTful service running on port 5000.

> :point_right: Start by running the preconfigured [docker-compose environment](#deployment) and reading [project README](ff-metric-docker/README.md).


## Ontology server

Ontology server (`/ontology-server`) exposes named graphs in the [Cliopatria](http://cliopatria.swi-prolog.org/home) installation over HTTP in RDF format via an Nginx server.

> :point_right: Start by following the [Getting Started section](ontology-server/README.md#getting-started) in the README.


## Optic Docker

Optic Docker (`/optic-docker`) is a project that packages an alternative [OPTIC planner](https://nms.kcl.ac.uk/planning/software/optic.html) via Docker.

> **NB!** This project does not expose OPTIC via REST (yet)!

> :point_right: Start by reading the [README](optic-docker/README.md).


## PDDL Examples

PDDL Examples (`/pddl-examples`) �s a folder with example PDDL files (domain and problem files), just for learning and testing purposes of the planners.


## PDDL Planner   - deprecated code that has been merged into Metric-FF Docker (`/ff-metric-docker`)

PDDL Planner (`/planner`) exposes PDDL planners via a REST API (so does the [Metric-FF Docker](#metric-ff-docker), but using dockerised Metric-FF). Theoretically supports both OPTIC and Metric-FF, but only implements support for Metric-FF. Requires local installation of Metric-FF (see `ff_path` in [plannerService.py](planner/plannerService.py)).

> :point_right: Start by using [Metric-FF Docker](#metric-ff-docker) and consult with Costa and Swarup if you need to touch this project.


## Planner Reasoner

Planner Reasoner (`/planner_reasoner`) uses an [internal planning ontology](planner_reasoner/rdf/base/pp.ttl) to transform the *warehouse state* (given in a form that adheres to the [warehouse ontology](planner_reasoner/rdf/base/warehouse_domain.ttl)) into the PDDL problem and domain files
.

> :point_right: Start by reading about the [`generate_pddl/2` predicate](planner_reasoner/lib/planner_reasoner.pl).

> **NB!** The project is called a *reasoner* because it is using Prolog reasoning facilities, it does not do any reasoning over the ontology and does not produce any inferred triples (as of now).
