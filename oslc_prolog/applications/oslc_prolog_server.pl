:- module(oslc_prolog_server, []).

:- use_module(library(http/thread_httpd)).
:- use_module(library(http/http_dispatch)).
:- use_module(library(http/http_files)).
:- use_module(library(http/http_parameters)).
:- use_module(library(http/json)).
:- use_module(library(http/http_json)).
:- use_module(library(http/http_error)).
:- use_module(library(broadcast)).

:- setting(oslc_prolog_server:base_uri, atom, 'http://localhost:3020/oslc/', 'Base URI').

:- http_handler('/oslc/', dispatcher, [prefix]).

:- listen(settings(changed(oslc_prolog_server:base_uri,Old,New)), (
     uri_components(New, uri_components(_,_,NewPath,_,_)),
     (  sub_atom(NewPath,_,1,0,'/')
     -> http_handler(NewPath, dispatcher, [prefix]),
        uri_components(Old, uri_components(_,_,OldPath,_,_)),
        http_delete_handler(path(OldPath))
     ;  atom_concat(New, '/', NewGood),
        set_setting(oslc_prolog_server:base_uri, NewGood)
     )
   )).

atoms_strings([], []) :- !.
atoms_strings([H|T], [SH|ST]) :-
    atom_string(H, SH),
    atoms_strings(T, ST).

dispatcher(Request) :-
  ( (
      member(protocol(Protocol), Request),
      member(host(Host), Request),
      member(port(Port), Request),
      member(path(Path), Request),
      atomic_list_concat([Protocol,'://',Host,':',Port,Path], '', Uri),
      setting(oslc_prolog_server:base_uri, Prefix),
      atom_concat(Prefix,ServicePath,Uri),
      split_string(ServicePath, '/', '/', Parts),
      dispatch(Request, Parts)
    )
  ->  true
  ; format('Status: 404~n~n')
  ).

dispatch(_, Parts) :-
  format('Status: 200~n'),
  format('Content-type: text/plain~n~n'),
  format('Parts = ~w~n', [Parts]).


% rdf_save(stream(current_output), [graph(mygraph)]).
% rdf_save_turtle(stream(current_output), [graph(mygraph)]).
% current_output(Out), rdf_save_subject(Out, 'http://my.com/catalog', [nsmap([])]).
