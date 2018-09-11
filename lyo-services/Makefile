.PHONY: build push

build:
	# to avoid copying /target to a Docker context
	# TODO .dockerignore because Gitlab CI runner does not have Maven by default
	# mvn -f lyo-webapp-parent/pom.xml clean
	# docker build . -f lyo.Dockerfile   -t scott-lyo-deps
	docker build -f maven.Dockerfile -t scott/maven-base .
	docker build -f sandbox.Dockerfile -t scott/sandbox-base .
	docker build -f twin.Dockerfile -t scott/webapp-twin .
	docker build -f whc.Dockerfile  -t scott/webapp-whc .
	docker build -f executor.Dockerfile  -t scott/webapp-executor .

push:
	docker push scott/lyo-deps
	docker push scott/maven-base
	docker push scott/webapp-twin
	docker push scott/webapp-whc
	docker push scott/webapp-executor

rebuild-lyo:
	docker build . -f lyo.Dockerfile   -t scott/scott-lyo-deps
