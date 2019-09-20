.PHONY: build push

build:
	docker build \
		--build-arg PUBLIC_HOST=localhost \
		--build-arg PUBLIC_PORT=3020 \
		--build-arg PREFIX_PATH=/ \
		--build-arg EXPOSED_PREFIXES=* \
		-t scott/planner-reasoner .

push:
	docker image tag scott/planner-reasoner ${CI_REGISTRY_IMAGE}/planner-reasoner:${CI_COMMIT_REF_NAME}
	docker push ${CI_REGISTRY_IMAGE}/planner-reasoner:${CI_COMMIT_REF_NAME}
