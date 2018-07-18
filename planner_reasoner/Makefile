.PHONY: build push

build:
	docker build \
		--build-arg PUBLIC_HOST=localhost \
		--build-arg PUBLIC_PORT=3020 \
		--build-arg PREFIX_PATH=/ \
		--build-arg EXPOSED_PREFIXES=* \
		-t ${CI_REGISTRY_IMAGE}/scott-planner-reasoner .

push:
	docker push ${CI_REGISTRY_IMAGE}/scott-planner-reasoner
