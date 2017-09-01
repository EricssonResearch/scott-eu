# Ontology server

A docker-compose project that exposes OSLC resources via [ClioPatria](http://cliopatria.swi-prolog.org/home) installation on top of [SWI-Prolog](http://swi-prolog.org) using REST through an Nginx reverse proxy. In uses OSLC library for Prolog from the scott-eu master branch.

## Getting started

1. Decide which hostname the nginx will run on. You can use `localhost` for the development or any other hostname that resolves to your machine's IP (run `hostname` to see your reverse PTR record, should be *blabla.ki.sw.ericsson.se*).
1. Build the `nginx` image with the right hostname (if you omit the parameters, `localhost` will be used), and port (`80` by default):

  `docker-compose build --build-arg PUBLIC_HOST=localhost PUBLIC_PORT=80 nginx`

3. Build the `cliopatria` image with the same parameters for public hostname and port (same defaults as for nginx; this is needed to tell ClioPatria to use correct redirects in UI, e.g. during login procedures). In addition set base URI parameter to the following value: `http://<PUBLIC_HOST>:<PUBLIC_PORT/`, and optionally specify the RDF prefixes to be exposed (all \* is default):

  `docker-compose build --build-arg PUBLIC_HOST=localhost PUBLIC_PORT=80 BASE_URI=http://localhost/ EXPOSED_PREFIXES=oslc,oslc_shapes cliopatria`

4. Run the composed set of images:

  `docker-compose up`

5. ClioPatria UI is at private address `http://localhost:3020`, it is not exposed via nginx.

   Log into ClioPatria via http://localhost:3020 using `admin` login and password that you got from Leo via Slack.

6. Use `http://localhost/<prefix>/<resource>` for REST queries towards OSLC resource `prefix:resource`, e.g. the following request will return Turtle representation of resource `oslc:Resource`. See http://open-services.net/bin/view/Main/OslcCoreSpecification for other methods.

  `curl -X GET -H Accept:application/turtle http://localhost/oslc/Resource`


