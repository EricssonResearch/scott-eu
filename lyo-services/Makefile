.PHONY: build push

build:
	docker build -f sandbox.Dockerfile -t scott/sandbox-base .
	docker build -f twin.Dockerfile -t scott/webapp-twin .
	docker build -f whc.Dockerfile  -t scott/webapp-whc .
	docker build -f executor.Dockerfile  -t scott/webapp-executor .

build-twin:
	docker build -f sandbox.Dockerfile -t scott/sandbox-base .
	docker build -f twin.Dockerfile -t scott/webapp-twin .

push:
	# docker push scott/sandbox-base
	docker push scott/webapp-twin
	docker push scott/webapp-whc
	docker push scott/webapp-executor
