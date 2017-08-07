:- module(conf_planner_reasoner, []).

/** <module> Planner Reasoner
*/

:- use_module(library(semweb/rdf_library)).
:- rdf_attach_library(planner_reasoner(rdf)).

:- rdf_load_library(pp).
:- rdf_load_library(pp_shapes).
:- rdf_load_library(warehouse_domain).
:- rdf_load_library(warehouse_problem).

:- use_module(library(planner_reasoner)).

%:- use_module(planner_reasoner(applications/planner_reasoner_server), []).
