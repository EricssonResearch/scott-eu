FROM scott/sandbox-base:latest as build

FROM flink:1.8

# https://github.com/apache/flink/blob/release-1.8/flink-container/docker/Dockerfile#L38
COPY --from=build /build/app/flink-trs-twin/target/flink-*.jar /opt/job.jar
