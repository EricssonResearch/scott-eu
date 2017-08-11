:- module(planner_reasoner_server, []).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdfs)).
:- use_module(library(oslc)).
:- use_module(library(oslc_rdf)).
:- use_module(library(oslc_dispatch)).
:- use_module(library(oslc_error)).
:- use_module(library(pddl_generator)).

:- oslc_post(planner:planCreationFactory, plan_creation_factory).

oslc_dispatch:serializer(text/'x-pddl', turtle).

plan_creation_factory(Context) :-
  once((
    Context.content_type == text/'x-pddl',
    with_output_to(string(String), (
      current_output(Out),
      forall(
        (
          rdf(Resource, rdf:type, pddl:'Domain', Context.graph_in)
        ; rdf(Resource, rdf:type, pddl:'Problem', Context.graph_in)
        ), (
          generate_pddl(Resource, Out),
          nl
        )
      )
    )),
    response(200),
    write(String)
  ; throw(response(400))
  )).
