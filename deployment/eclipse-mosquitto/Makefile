.PHONY: build push

build:
	docker build -f Dockerfile -t scott/eclipse-mosquitto .

push:
	docker image tag scott/eclipse-mosquitto ${CI_REGISTRY_IMAGE}/scott/eclipse-mosquitto:$(CI_COMMIT_REF_NAME)
	docker push ${CI_REGISTRY_IMAGE}/scott/eclipse-mosquitto:$(CI_COMMIT_REF_NAME)
