.PHONY: build up

build:
	(cd lyo-webapp-parent && mvn clean package)
	(cd deployment 				&& docker-compose build)

up: build
	(cd deployment 				&& docker-compose up)
