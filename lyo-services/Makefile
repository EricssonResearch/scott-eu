.PHONY: build push

build:
	docker build -f sandbox.Dockerfile -t scott/sandbox-base .
	(cd flink-trs-twin && mvn clean package)
	(cd flink-docker && ./build.sh --job-jar ../flink-trs-twin/target/flink-*.jar --from-release --flink-version 1.8.0 --scala-version 2.12 --image-name scott/flink-trs-twin)
	docker build -f whc.Dockerfile  -t scott/webapp-whc .
	docker build -f twin-robot.Dockerfile -t scott/webapp-twin-robot .
	docker build -f twin-shelf.Dockerfile -t scott/webapp-twin-shelf .
	docker build -f executor.Dockerfile  -t scott/webapp-executor .
	docker build -f svc-location.Dockerfile  -t scott/webapp-svc-location .
	# docker build -f flink-trs.Dockerfile  -t scott/flink-trs-twin .

build-twin-robot:
	docker build -f sandbox.Dockerfile -t scott/sandbox-base .
	docker build -f twin-robot.Dockerfile -t scott/webapp-twin-robot .

build-whc:
	docker build -f sandbox.Dockerfile -t scott/sandbox-base .
	docker build -f whc.Dockerfile -t scott/webapp-whc .

push:
	docker image tag scott/webapp-twin-robot ${CI_REGISTRY_IMAGE}/webapp-twin-robot:${CI_COMMIT_REF_NAME}
	docker push ${CI_REGISTRY_IMAGE}/webapp-twin-robot:${CI_COMMIT_REF_NAME}

	docker image tag scott/webapp-twin-shelf ${CI_REGISTRY_IMAGE}/webapp-twin-shelf:${CI_COMMIT_REF_NAME}
	docker push ${CI_REGISTRY_IMAGE}/webapp-twin-shelf:${CI_COMMIT_REF_NAME}

	docker image tag scott/webapp-whc ${CI_REGISTRY_IMAGE}/webapp-whc:${CI_COMMIT_REF_NAME}
	docker push ${CI_REGISTRY_IMAGE}/webapp-whc:${CI_COMMIT_REF_NAME}

	docker image tag scott/webapp-executor ${CI_REGISTRY_IMAGE}/webapp-executor:${CI_COMMIT_REF_NAME}
	docker push ${CI_REGISTRY_IMAGE}/webapp-executor:${CI_COMMIT_REF_NAME}

	docker image tag scott/webapp-svc-location ${CI_REGISTRY_IMAGE}/webapp-svc-location:${CI_COMMIT_REF_NAME}
	docker push ${CI_REGISTRY_IMAGE}/webapp-svc-location:${CI_COMMIT_REF_NAME}

	# TODO flink
