:- module(oslc_example, []).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdf_library)).
:- use_module(library(oslc)).
:- use_module(library(oslc_dispatch)).

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslc_example).

:- oslc_get(oslc_example:leoResourceShape, handle_get).

handle_get(_IRI, _GraphOut) :-
  format('Status: 416~n~n').
