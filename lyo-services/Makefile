.PHONY: build push

build:
	docker build -f sandbox.Dockerfile -t scott/sandbox-base .
	docker build -f twin-robot.Dockerfile -t scott/webapp-twin-robot .
	docker build -f whc.Dockerfile  -t scott/webapp-whc .
	docker build -f executor.Dockerfile  -t scott/webapp-executor .

build-twin:
	docker build -f sandbox.Dockerfile -t scott/sandbox-base .
	docker build -f twin-robot.Dockerfile -t scott/webapp-twin-robot .

build-whc:
	docker build -f sandbox.Dockerfile -t scott/sandbox-base .
	docker build -f whc.Dockerfile -t scott/webapp-whc .

push:
	docker image tag scott/webapp-twin-robot ${CI_REGISTRY_IMAGE}/scott/webapp-twin-robot:$(CI_COMMIT_REF_NAME)
	docker push ${CI_REGISTRY_IMAGE}/scott/webapp-twin-robot:$(CI_COMMIT_REF_NAME)
	docker image tag scott/webapp-whc ${CI_REGISTRY_IMAGE}/scott/webapp-whc:$(CI_COMMIT_REF_NAME)
	docker push ${CI_REGISTRY_IMAGE}/scott/webapp-whc:$(CI_COMMIT_REF_NAME)
	docker image tag scott/webapp-executor ${CI_REGISTRY_IMAGE}/scott/webapp-executor:$(CI_COMMIT_REF_NAME)
	docker push ${CI_REGISTRY_IMAGE}/scott/webapp-executor:$(CI_COMMIT_REF_NAME)
