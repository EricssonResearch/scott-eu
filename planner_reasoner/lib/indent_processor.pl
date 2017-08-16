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

:- module(indent_processor, [
  insert_indents/2,
  nonempty/1
]).

insert_indents(In, Out) :-
  insert_indents(In, Out, 0, true).

insert_indents(In, Out, Indent, ApplyIndent) :-
  flatten(In, Flat),
  insert_indents0(Flat, Indented, Indent, ApplyIndent),
  flatten(Indented, Out).

insert_indents0([], [], _, _) :- !.

insert_indents0([indent(N, H)|T], Out, Indent, _) :- !,
  ( nonempty(H)
  -> NewIndent is Indent + N,
     insert_indents(H, H3, NewIndent, true),
     append(["\n"|H3], T2, Out)
  ; Out = [T2]
  ),
  insert_indents0(T, T2, Indent, true).

insert_indents0([H|T], [H2|T2], Indent, ApplyIndent) :-
  ( ApplyIndent == true
  -> format(string(H2), '~*|~w', [Indent, H])
  ; H2 = H
  ),
  ( string_concat(_, "\n", H)
  -> NewApplyIndent = true
  ; NewApplyIndent = false
  ),
  insert_indents0(T, T2, Indent, NewApplyIndent).

nonempty(L) :-
  length(L, N),
  N > 0.
