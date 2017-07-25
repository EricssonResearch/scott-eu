:- module(oslc_dispatch, [
  oslc_get/2,
  oslc_get/3,
  oslc_post/2,
  oslc_post/3,
  oslc_put/2,
  oslc_put/3,
  oslc_delete/2,
  oslc_delete/3,
  add_handler/4,
  delete_handler/2,
  dispatch/4
]).

:- meta_predicate(oslc_get(+, 3)).
:- meta_predicate(oslc_get(+, 3, +)).
:- meta_predicate(oslc_post(+, 4)).
:- meta_predicate(oslc_post(+, 4, +)).
:- meta_predicate(oslc_put(+, 4)).
:- meta_predicate(oslc_put(+, 4, +)).
:- meta_predicate(oslc_delete(+, 2)).
:- meta_predicate(oslc_delete(+, 2, +)).

:- dynamic handler/4.

oslc_get(IRISpec, Predicate) :-
  oslc_get(IRISpec, Predicate, 1000).

oslc_get(IRISpec, Predicate, Priority) :-
  add_handler(get, IRISpec, Predicate, Priority).

oslc_post(IRISpec, Predicate) :-
  oslc_post(IRISpec, Predicate, 1000).

oslc_post(IRISpec, Predicate, Priority) :-
  add_handler(post, IRISpec, Predicate, Priority).

oslc_put(IRISpec, Predicate) :-
  oslc_put(IRISpec, Predicate, 1000).

oslc_put(IRISpec, Predicate, Priority) :-
  add_handler(put, IRISpec, Predicate, Priority).

oslc_delete(IRISpec, Predicate) :-
  oslc_delete(IRISpec, Predicate, 1000).

oslc_delete(IRISpec, Predicate, Priority) :-
  add_handler(delete, IRISpec, Predicate, Priority).

add_handler(Method, IRISpec, Predicate, Priority) :-
  must_be(atom, Method),
  must_be(ground, IRISpec),
  must_be(ground, Predicate),
  must_be(integer, Priority),
  retractall(handler(Method, IRISpec, _, Priority)),
  assertz(handler(Method, IRISpec, Predicate, Priority)).

delete_handler(Method, IRISpec) :-
  forall(
    handler(Method, IRISpec, Predicate, Priority),
    retractall(handler(Method, IRISpec, Predicate, Priority))
  ).

dispatch(Request, Prefix:ResourceSegments, GraphIn, GraphOut) :-
  member(method(Method), Request),
  atomic_list_concat(ResourceSegments, '/', Resource),
  findall(
    handler(Method, IRI, Predicate, Priority), (
      handler(Method, ISPrefix:ISResource, Predicate, Priority),
      match_wildcard(ISPrefix, Prefix, ISResource, Resource),
      rdf_global_id(Prefix:Resource, IRI)
    ),
    Handlers
  ),
  predsort(handler_compare, Handlers, [handler(Method, IRI, Predicate, _)|_]),
  dispatch_to_handler(Method, Predicate, Request, IRI, GraphIn, GraphOut).

match_wildcard(ISPrefix, Prefix, ISResource, Resource) :-
  once((
    ISPrefix == Prefix
  ; ISPrefix == *
  )),
  once((
    ISResource == Resource
  ; ISResource == *
  )).

handler_compare(<, handler(_,_,_,P1), handler(_,_,_,P2)) :-
  P1 >= P2.
handler_compare(>, handler(_,_,_,P1), handler(_,_,_,P2)) :-
  P1 < P2.

dispatch_to_handler(get, Module:Predicate, Request, IRI, _, GraphOut) :-
  T =.. [Predicate, Request, IRI, GraphOut],
  call(Module:T).

dispatch_to_handler(post, Module:Predicate, Request, IRI, GraphIn, GraphOut) :-
  T =.. [Predicate, Request, IRI, GraphIn, GraphOut],
  call(Module:T).

dispatch_to_handler(put, Module:Predicate, Request, IRI, GraphIn, GraphOut) :-
  T =.. [Predicate, Request, IRI, GraphIn, GraphOut],
  call(Module:T).

dispatch_to_handler(delete, Module:Predicate, Request, IRI, _, _) :-
  T =.. [Predicate, Request, IRI],
  call(Module:T).
