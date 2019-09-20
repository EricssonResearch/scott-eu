.PHONY: build push

build:
	(cd flink-trs-twin && mvn clean package)
	(cd flink-docker && ./build.sh --job-jar ../flink-trs-twin/target/flink-*.jar --from-release --flink-version 1.8.1 --scala-version 2.12 --image-name scott/flink-trs-twin)

push:
	echo "Not ready yet"
	exit 1
