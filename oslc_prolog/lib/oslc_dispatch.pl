/*
Copyright 2017 Ericsson AB

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

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
  dispatch/1,
  response/1,
  response/2,
  select_acceptable_content_type/2
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/turtle)).

:- multifile serializer/2.
:- multifile serialize_response/3.
:- multifile error_message/2.

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
    handler(Method, IRISpec, Handler, Priority),
    retractall(handler(Method, IRISpec, Handler, Priority))
  ).

dispatch(Context) :-
  _{ iri_spec: Prefix:ResourceSegments,
       method: Method } :< Context,
  atomic_list_concat(ResourceSegments, '/', Resource),
  findall(handler(Handler, Priority), (
    handler(Method, ISPrefix:ISResource, Handler, Priority),
    match_wildcard(ISPrefix, Prefix, ISResource, Resource)
  ), Handlers),
  predsort(handler_compare, Handlers, SortedHandlers),
  rdf_global_id(Prefix:Resource, IRI),
  NewContext = Context.put(iri, IRI).put(iri_spec, Prefix:Resource),
  dispatch_to_handlers(NewContext, SortedHandlers).

dispatch_to_handlers(Context, [handler(Module:Predicate, _)|T]) :-
  once((
    Term =.. [Predicate, Context],
    call(Module:Term)
  ; ( nonvar(Context.graph_out)
    -> rdf_retractall(_, _, _, Context.graph_out)
    ; true
    ),
    dispatch_to_handlers(Context, T)
  )).

match_wildcard(ISPrefix, Prefix, ISResource, Resource) :-
  once((
    ISPrefix == Prefix
  ; ISPrefix == *
  )),
  once((
    ISResource == Resource
  ; ISResource == *
  )).

handler_compare(<, handler(_,P1), handler(_,P2)) :-
  P1 >= P2.
handler_compare(>, handler(_,P1), handler(_,P2)) :-
  P1 < P2.

response(StatusCode) :-
  format("Status: ~w~n~n", [StatusCode]).

response(StatusCode, Headers) :-
  format_headers(Headers, HeadersString),
  format("Status: ~w~n~w~n", [StatusCode, HeadersString]).

format_headers(Headers, Output) :-
  maplist(format_header, Headers, O),
  atomics_to_string(O, Output).

format_header(H, H2) :-
  H =.. [Header, Value],
  format(atom(H2), '~w: ~w~n', [Header, Value]).

serializer(application/'rdf+xml', rdf).
serializer(application/'turtle', turtle).
serializer(text/'rdf+xml', rdf).
serializer(application/'x-turtle', turtle).
serializer(text/'turtle', turtle).

serialize_response(Out, Graph, rdf) :-
  rdf_save(stream(Out), [graph(Graph)]).

serialize_response(Out, Graph, turtle) :-
  rdf_save_turtle(Out, [graph(Graph), comment(false), silent(true), tab_distance(0)]).

error_message(400, 'Bad request').
error_message(404, 'Not found').
error_message(405, 'Method not allowed').
error_message(406, 'Not acceptable').
error_message(411, 'Content length required').
error_message(412, 'Precondition failed').
error_message(415, 'Unsupported media type').
error_message(_, 'No message').

select_acceptable_content_type(Request, ContentType) :-
  member(accept(Accept), Request), % fetch accept header
  predsort(accept_compare, Accept, AcceptSorted), % sort requested content types according to qualities
  select_content_type(AcceptSorted, ContentType). % select the best matching content type

select_content_type([], _) :- fail.
select_content_type([media(H,_,_,_)|T], ContentType) :- % go through all requested content types
  ( serializer(H, _)
  -> !, ContentType = H % if content type is */* then H becomes _VAR/_VAR and matches application/'rdf+xml'
  ; select_content_type(T, ContentType)
  ).

accept_compare(<, media(_,_,W1,_), media(_,_,W2,_)) :- % sort qualities in reverse order - biggest first
  W1 >= W2.
accept_compare(>, media(_,_,W1,_), media(_,_,W2,_)) :-
  W1 < W2.
