:- module(oslc_dispatch, [
  oslc_get/2,
  oslc_get/3,
  oslc_post/2,
  oslc_post/3,
  oslc_put/2,
  oslc_put/3,
  oslc_delete/2,
  oslc_delete/3,
  dispatch/4
]).

:- meta_predicate(oslc_get(+, 2)).
:- meta_predicate(oslc_get(+, 2, +)).
:- meta_predicate(oslc_post(+, 3)).
:- meta_predicate(oslc_post(+, 3, +)).
:- meta_predicate(oslc_put(+, 3)).
:- meta_predicate(oslc_put(+, 3, +)).
:- meta_predicate(oslc_delete(+, 1)).
:- meta_predicate(oslc_delete(+, 1, +)).

oslc_get(IRI, Predicate) :-
  oslc_get(IRI, Predicate, 1000).

oslc_get(IRI, Predicate, Priority) :-
  add_handler(get, IRI, Predicate, Priority).

oslc_post(IRI, Predicate) :-
  oslc_post(IRI, Predicate, 1000).

oslc_post(IRI, Predicate, Priority) :-
  add_handler(post, IRI, Predicate, Priority).

oslc_put(IRI, Predicate) :-
  oslc_put(IRI, Predicate, 1000).

oslc_put(IRI, Predicate, Priority) :-
  add_handler(put, IRI, Predicate, Priority).

oslc_delete(IRI, Predicate) :-
  oslc_delete(IRI, Predicate, 1000).

oslc_delete(IRI, Predicate, Priority) :-
  add_handler(delete, IRI, Predicate, Priority).

add_handler(Method, IRI, Predicate, Priority) :-
  format(current_output, '~w: ~w -> ~w [~w]~n', [Method, IRI, Predicate, Priority]).


dispatch(Prefix:Resource, Request, GraphIn, GraphOut) :-
  format('Status: 200~n'),
  format('Content-type: text/plain; charset=utf-8~n~n'),
  format('~w:~w ~w ~w ~w~n', [Prefix, Resource, Request, GraphIn, GraphOut]).
