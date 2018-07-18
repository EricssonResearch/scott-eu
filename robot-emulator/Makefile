.PHONY: build push

build:
	docker build -t ${CI_REGISTRY_IMAGE}/scott-robot-emulator .

push:
	docker push ${CI_REGISTRY_IMAGE}/scott-robot-emulator
