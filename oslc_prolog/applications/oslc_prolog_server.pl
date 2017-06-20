:- module(oslc_prolog_server, []).

:- use_module(library(http/thread_httpd)).
:- use_module(library(http/http_dispatch)).
:- use_module(library(http/http_files)).
:- use_module(library(http/http_parameters)).
:- use_module(library(http/json)).
:- use_module(library(http/http_json)).
:- use_module(library(http/http_error)).

:- http_handler('/metamodel/', dispatcher, [prefix]).

atoms_strings([], []) :- !.
atoms_strings([H|T], [SH|ST]) :-
    atom_string(H, SH),
    atoms_strings(T, ST).

dispatcher(Request) :-
  ( ( member(path_info(P), Request),
      split_string(P, '/', '/', Parts),
      atoms_strings([H|T], Parts),
      dispatch(H, T, Request),
      format('Status: 200~n'),
      format('Content-type: text/plain~n~n'),
      format('OK~n')
    )
  ->  true
  ; format('Status: 404~n~n')
  ).

dispatch(message, [], _).
