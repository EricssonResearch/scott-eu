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

:- module(pddl_parser, [
  parse_pddl/2
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(grammar_utils)).

parse_pddl(String, Graph) :-
  setup_call_cleanup(
    rdf0_set_graph(Graph), (
      string_codes(String, Codes),
      phrase(lexer(Out), Codes, []),
      print(Out)
    ),
    rdf0_unset_graph(Graph)
  ).

lexer(T) --> [C], { code_type(C, space) }, !, lexer(T).
lexer([H|T]) --> element(H), !, lexer(T).
lexer([]) --> [].

element('(') --> [40].
element(')') --> [41].
element(key(T)) --> [58], word(T).
element(var(T)) --> [63], word(T).
element(T) --> word(T).

word(W) --> token(T), { T \== [], atom_codes(W, T) }.

token([C|T]) --> [C], { C \== -1,
                        \+ code_type(C, space),
                        \+ member(C, [40,41,58,63])
                      }, !, token(T).
token([]) --> [].

test2 :-
  parse_pddl("(define (domain adl-blocksworld)
        (:requirements :adl :equality)
        (:types location block)
        (:predicates (on ?x ?y - (either block location))
                   (clear ?x - (either block location))
        )
        (:functions (moved ?m - (either block location))
                    (total-moved))
        (:action move
           :parameters (?b - block ?x ?y - (either block location))
           :precondition
              (and
                (not(= ?b ?y))
                (clear ?b)
                (on ?b ?x)
               (clear ?y)
              )
           :effect
              (and
                (on ?b ?y)
                (not (on ?b ?x))
                (clear ?x)
                (increase (moved ?b) 1)
                (increase (total-moved) 1)
                (when
                  (not (= ?y table))
                  (not (clear ?y))
                )
              )
        )
      )

    (define (problem adl-blocksworld-problem)
      (:domain adl-blocksworld)
      (:objects a b c - block table - location)
      (:init
        (on b table) (on a table) (on c a)
        (clear b) (clear c) (clear table)
      )
      (:goal
        (or
          (on b c)
          (on c b)
        )
      )
      (:metric minimize total-time)
    )

      ", user).
