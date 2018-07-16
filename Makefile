.PHONY: build up

build:
	(cd lyo-webapp-parent && mvn clean package)
	(cd deployment 				&& docker-compose build)

up: build
	(cd deployment 				&& docker-compose up)

up-quick:
	(cd deployment 				&& docker-compose build)
	(cd deployment 				&& docker-compose up)

swarm-restart:
	(cd deployment 				&& docker-compose build)
	(cd deployment 				&& docker swarm init) | true
	(cd deployment 				&& docker stack rm scott) | true
	(cd deployment 				&& docker stack deploy -c docker-compose.yml scott)