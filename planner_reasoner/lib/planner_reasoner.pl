:- module(planner_reasoner, [
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdf_library)).

:- rdf_register_prefix(pp, 'http://ontology.cf.ericsson.net/planning_problem#').

:- rdf_attach_library(planner_reasoner(rdf)).
:- rdf_load_library(pp).
:- rdf_load_library(wd).
