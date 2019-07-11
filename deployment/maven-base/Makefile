.PHONY: build push

build:
	#(docker run -it --rm -v "$$PWD":/src -v "$$PWD/m2_temp":/root/.m2/repository -w /src maven:3-jdk-8 mvn clean package -f lyo-webapp-parent/pom.xml -q -B) || true
	docker build -f maven.Dockerfile -t scott/maven-base .

push:
	docker image tag scott/maven-base ${CI_REGISTRY_IMAGE}/maven-base:$(CI_COMMIT_REF_NAME)
	docker push ${CI_REGISTRY_IMAGE}/maven-base:$(CI_COMMIT_REF_NAME)
