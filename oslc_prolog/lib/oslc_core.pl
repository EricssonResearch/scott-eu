:- module(oslc_core, []).

:- use_module(library(oslc)).
:- use_module(library(oslc_dispatch)).

:- oslc_get(*:*, handle_get, 0).

handle_get(IRI, GraphOut) :-
  copy_resource(IRI, IRI, rdf, rdf(GraphOut), [inline(rdf)]).
