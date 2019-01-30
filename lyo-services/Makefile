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
	# docker push scott/sandbox-base
	docker push scott/webapp-twin-robot
	docker push scott/webapp-whc
	docker push scott/webapp-executor
