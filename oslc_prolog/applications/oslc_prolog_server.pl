:- module(oslc_prolog_server, []).

:- use_module(library(http/thread_httpd)).
:- use_module(library(http/http_dispatch)).
:- use_module(library(broadcast)).
:- use_module(library(semweb/turtle)).
:- use_module(library(semweb/rdf_persistency)).

:- setting(oslc_prolog_server:base_uri, atom, 'http://localhost:3020/oslc/', 'Base URI').
:- setting(oslc_prolog_server:served_prefix, atom, '', 'Served Prefix').

:- http_handler('/oslc/', dispatcher, [prefix]).

:- listen(settings(changed(oslc_prolog_server:base_uri, Old, New)), (
     uri_components(New, uri_components(Schema, Authority, NewPath, _, _)),
     (  sub_atom(NewPath, _, 1, 0, '/') % if new base URI is ending with '/'
     -> http_handler(NewPath, dispatcher, [prefix]), % set a handler for new base URI
        uri_components(Old, uri_components(_, _, OldPath, _, _)),
        http_delete_handler(path(OldPath)) % remove old handler
     ;  atomic_list_concat([Schema, '://', Authority, NewPath, '/'], '', NewSetting), % append '/' to the new URI
        set_setting(oslc_prolog_server:base_uri, NewSetting) % do set setting again, it will call this listener
     )
   )).

dispatcher(Request) :-
  ( (
      member(protocol(Protocol), Request),
      member(host(Host), Request),
      member(port(Port), Request),
      member(path(Path), Request),
      atomic_list_concat([Protocol, '://', Host, ':', Port,  Path], '', Uri), % determine URI called
      setting(oslc_prolog_server:base_uri, Prefix),
      atom_concat(Prefix, ServicePath, Uri), % check if URI called start with base URI
      dispatch(Request, ServicePath) % call dispatch with computed suffix
    )
  ->  true
  ; format('Status: 404~n~n')
  ).

dispatch(Request, ServicePath) :-
  current_output(Out),
  setting(oslc_prolog_server:served_prefix, Prefix), % fetch served RDF prefix from settings
  atom_concat(Prefix, ServicePath, Resource), % compute served RDF resource URI
  setup_call_cleanup(make_graph(Resource, Graph), ( % create and populate new temporary RDF graph
    rdf_graph_property(Graph, triples(Triples)),
    Triples > 0,
    select_content_type(Request, ContentType), % compute content-type to serve based on request
    ( ContentType == nil
    -> format('Status: 415~n~n') % requested content-type cannot be served
    ;  format('Status: 200~n'),
       format('Content-type: ~w; charset=utf-8~n~n', [ContentType]),
       save_graph(Out, Graph, ContentType) % serialize temporary RDF graph to the response
    )
  ), rdf_unload_graph(Graph)). % regardless of the result remove temporary RDF graph

make_graph(Resource, Graph) :-
  uuid(Graph), % generate unique identifier for the temporary RDF graph
  rdf_create_graph(Graph),
  rdf_persistency(Graph, false), % do not persist (keep only in RAM)
  forall(rdf(Resource, P, O), rdf_assert(Resource, P, O, Graph)). % poor man's populate method

uuid(Id) :-
  Max is 1<<128,
  random_between(0, Max, Num),
  atom_number(Id, Num).

select_content_type(Request, ContentType) :-
  member(accept(Accept), Request),
  predsort(accept_compare, Accept, AcceptSorted), % sort requested content types according to qualities
  select_content_type0(AcceptSorted, ContentType).

select_content_type0([], nil) :- !.
select_content_type0([media(H,_,_,_)|T], ContentType) :- % go through all requested content types
  ( member(H, [application/'rdf+xml', text/'turtle', application/'x-turtle'])
  -> ContentType = H % if content type is */* then H becomes _VAR/_VAR and matches application/'rdf+xml'
  ; select_content_type0(T, ContentType)
  ).

accept_compare(<, media(_,_,W1,_), media(_,_,W2,_)) :- % sort qualities in reverse order - biggest first
  W1 > W2.
accept_compare(<, _, _).

save_graph(Out, Graph, application/'rdf+xml') :-
  rdf_save(stream(Out), [graph(Graph)]).

save_graph(Out, Graph, text/'turtle') :-
  rdf_save_turtle(Out, [graph(Graph)]).

save_graph(Out, Graph, application/'x-turtle') :-
  rdf_save_turtle(Out, [graph(Graph)]).
