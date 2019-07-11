.PHONY: build push

build:
	docker build -t scott/robot-emulator .

push:
	docker image tag scott/robot-emulator ${CI_REGISTRY_IMAGE}/robot-emulator:$(CI_COMMIT_REF_NAME)
	docker push ${CI_REGISTRY_IMAGE}/robot-emulator:$(CI_COMMIT_REF_NAME)
