.PHONY: build push

build:
	docker build -f Dockerfile -t scott/jena-fuseki .

push:
	docker image tag scott/jena-fuseki ${CI_REGISTRY_IMAGE}/jena-fuseki:$(CI_COMMIT_REF_NAME)
	docker push ${CI_REGISTRY_IMAGE}/jena-fuseki:$(CI_COMMIT_REF_NAME)
