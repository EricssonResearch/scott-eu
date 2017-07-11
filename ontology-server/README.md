# Ontology server

A docker-compose project that exposes named graphs in the [Cliopatria](http://cliopatria.swi-prolog.org/home) installation over HTTP in RDF format via an Nginx server.

## Getting started

1. Decide which hostname the nginx will run on. You can use `localhost` for the development or any other hostname that resolves to your machine's IP (run `hostname` to see your reverse PTR record, should be *blabla.ki.sw.ericsson.se*).
1. Build the `nginx` image with the right hostname (if you omit the parameters, `localhost` will be used):

  `docker-compose build --build-arg ONTOLOGY_HOSTNAME=localhost nginx`
  
1. Run the composed set of images:

  `docker-compose up`

1. Visit http://localhost:3020 to set up Cliopatria.
1. visit `http://localhost/_suffix_`, to see the named graph contents.

In the command above, `_suffix_` stands for the part of the named graph URI after the hostname: `http://_hostname_/_suffix_`. To configure Cliopatria:

- Log into Cliopatria via http://localhost:3020 using `admin` login and password that you got from Leo via Slack.
- Upload [pp.ttl](planner_reasoner/rdf/base/pp.ttl) into named graph `http://_hostname_/planning_problem` via the http://localhost:3020/user/loadFile
- Upload [warehouse_domain.ttl](planner_reasoner/rdf/base/warehouse_domain.ttl) into named graph `http://_hostname_/warehouse_domain` via the http://localhost:3020/user/loadFile
- Open http://localhost:80/warehouse_domain. Nginx should return the named graph with the URI `http://localhost/` (URI base, unless you used another hostname) *plus* `warehouse_domain` (URI fragment from your request).

