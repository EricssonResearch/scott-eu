#!/bin/sh -xe

SERVICE=${1:-server}

case $SERVICE in
  server)
    java -jar ./leshan-server-demo.jar ;;
  bootstrap|bsserver)
 #   java -jar ./leshan-bsserver-demo.jar ;;
 # *)
    echo "Usage: <server|bootstrap>"
    exit 1
esac

