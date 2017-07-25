:- module(oslc_example, []).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdf_library)).
:- use_module(library(oslc)).
:- use_module(library(oslc_dispatch)).

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslc_example).

:- oslc_get(oslc_example:leoResource, handle_get).
:- oslc_post(oslc_example:leoResource, handle_post_put).
:- oslc_put(oslc_example:leoResource, handle_post_put).
:- oslc_delete(oslc_example:leoResource, handle_delete).

handle_get(Request, _IRI, GraphOut) :-
  get_time(T),
  member(user_agent(A), Request),
  atom_string(A, S),
  create_resource(oslc_example:leoResource, [oslc_example:leoResourceClass],
                 [title=S, created=T], rdf(GraphOut)).

handle_post_put(_Request, _IRI, _GraphIn, _GraphOut) :-
  format('Status: 417~n~n').

handle_delete(_Request, _IRI) :-
  format('Status: 418~n~n').
