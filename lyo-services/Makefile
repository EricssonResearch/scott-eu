.PHONY: build

all:
	# to avoid copying /target to a Docker context
	# TODO .dockerignore because Gitlab CI runner does not have Maven by default
	# mvn -f lyo-webapp-parent/pom.xml clean
	# docker build . -f lyo.Dockerfile   -t scott-lyo-deps
	docker build . -f twin.Dockerfile  -t scott-webapp-twin
	docker build . -f whc.Dockerfile   -t scott-webapp-whc
