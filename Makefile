.PHONY: build up build-twin-robot restart-twin

STACK ?= docker-compose.dev.yml

build:
	(cd planner_reasoner 		&&	make build)
	(cd deployment/maven-base 	&&	make build)
	(cd deployment/eclipse-mosquitto &&	make build)
	(cd deployment/jena-fuseki 	&&	make build)
	(cd lyo-services 		&&	make build)
	# (cd flink-jobs 			&&	make build)
	# (cd robot-emulator 		&&	make build)

push:
	(cd planner_reasoner		&&	make push)
	(cd deployment/maven-base 	&&	make push)
	(cd deployment/eclipse-mosquitto &&	make push)
	(cd deployment/jena-fuseki 	&&	make push)
	(cd lyo-services		&&	make push)
	#(cd flink-jobs 			&&	make push)
	(cd robot-emulator		&&	make push)

restart-swarm:
	(cd deployment	&& docker service rm `docker service ls --filter name=scott -q`) || true
	(cd deployment	&& docker stack deploy -c $(STACK) --prune scott)

restart-swarm-full:
	(cd deployment	&& docker swarm init) || true
	(cd deployment	&& docker service rm `docker service ls --filter name=scott -q` && sleep 1) || true
	(cd deployment  && docker stack rm scott && sleep 10) || true
	(cd deployment	&& docker stack deploy -c $(STACK) --prune scott)

build-twin-robot:
	(cd lyo-services 		&&	make build-twin-robot)

restart-twin-robot:
	(cd deployment	&& docker service rm `docker service ls --filter name=scott_sandbox-twin -q` && sleep 1) || true
	(cd deployment	&& docker stack deploy -c $(STACK) --prune scott)

build-whc:
	(cd lyo-services 		&&	make build-whc)

restart-whc:
	(cd deployment	&& docker service rm `docker service ls --filter name=scott_sandbox-whc -q` && sleep 1) || true
	(cd deployment	&& docker stack deploy -c $(STACK) --prune scott)
