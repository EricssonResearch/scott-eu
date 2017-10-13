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

:- module(oslc_error, [
  oslc_error/2
]).

:- use_module(library(semweb/rdf11)).

% FIXME: probably there is a better way to report errors...

oslc_error(Message, Arguments) :-
  error0(Message, Arguments, []).

error0(Message, [], Accum):-
  reverse(Accum, Arguments),
  format(atom(Error), Message, Arguments),
  throw(oslc_error(Error)).

error0(Message, [H|T], Accum) :-
  to_local_name(H, H2),
  error0(Message, T, [H2|Accum]).

to_local_name(H, H2) :-
  \+ is_list(H),
  ( rdf_is_resource(H)
  -> rdf_global_id(H2, H)
  ; ( H = ^^(H2, _)
    -> true
    ; H2 = H
    )
  ).

to_local_name([], []) :- !.

to_local_name([H|T], [H2|T2]) :-
  to_local_name(H, H2),
  to_local_name(T, T2).
