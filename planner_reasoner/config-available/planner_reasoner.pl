:- module(conf_planner_reasoner, []).

/** <module> Planner Reasoner
*/

:- use_module(library(semweb/rdf_library)).
:- rdf_attach_library(planner_reasoner(rdf)).

:- rdf_register_prefix(sh, 'http://www.w3.org/ns/shacl#').

:- rdf_load_library(pddl).
:- rdf_load_library(pddl_shapes).
:- rdf_load_library(pddl_domain_example).
:- rdf_load_library(pddl_problem_example).
:- rdf_load_library(planner).

:- use_module(library(pddl_generator)).

:- use_module(planner_reasoner(applications/planner_reasoner_server), []).
