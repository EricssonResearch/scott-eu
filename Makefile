.PHONY: build up build-twin restart-twin

build:
	(cd planner_reasoner 		&&	make build)
	(cd deployment/maven-base 	&&	make build)
	(cd lyo-services 		&&	make build)
	(cd robot-emulator 		&&	make build)

push:
	(cd planner_reasoner		&&	make push)
	(cd deployment/maven-base 	&&	make push)
	(cd lyo-services		&&	make push)
	(cd robot-emulator		&&	make push)

restart-swarm:
	(cd deployment	&& docker swarm init) || true
	(cd deployment	&& docker service rm `docker service ls --filter name=scott -q` && sleep 1) || true
	(cd deployment  && docker stack rm scott && sleep 15) || true
	(cd deployment	&& docker stack deploy -c docker-compose.yml --prune scott)

build-twin:
	(cd lyo-services 		&&	make build-twin)

restart-twin:
	(cd deployment	&& docker service rm `docker service ls --filter name=scott_sandbox-twin -q` && sleep 1) || true
	(cd deployment	&& docker stack deploy -c docker-compose.yml --prune scott)

build-whc:
	(cd lyo-services 		&&	make build-whc)

restart-whc:
	(cd deployment	&& docker service rm `docker service ls --filter name=scott_sandbox-whc -q` && sleep 1) || true
	(cd deployment	&& docker stack deploy -c docker-compose.yml --prune scott)
