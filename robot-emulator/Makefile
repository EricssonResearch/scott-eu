.PHONY: build push

build:
	docker build -t scott/robot-emulator .

push:
	docker image tag scott/robot-emulator ${CI_REGISTRY}/scott/robot-emulator
	docker push ${CI_REGISTRY}/scott/robot-emulator
