:- module(oslc_core, []).

:- use_module(library(oslc)).
:- use_module(library(oslc_dispatch)).

:- oslc_get((*):(*), handle_get, 0).
:- oslc_post((*):(*), handle_post, 0).

handle_get(Request, IRI, GraphOut) :-
  once(rdf(IRI, _, _)),
  ( member(search(Search), Request),
    member(('oslc.properties'=Properties), Search)
  -> Option = [properties(Properties)]
  ; Option = []
  ),
  copy_resource(IRI, IRI, rdf, rdf(GraphOut), [inline(rdf)|Option]).

handle_post(_Request, _IRI, _GraphIn, _GraphOut) :-
  true.
