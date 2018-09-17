This container just exposes the access to the `ff` command.

Example: to run `ff --help`:

    docker build -t metric-ff-bare .
    docker run -it -v "$(pwd)":/workdir -w /workdir metric-ff-bare --help
