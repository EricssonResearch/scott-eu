:- module(oslc_prolog_server, []).

:- use_module(library(http/thread_httpd)).
:- use_module(library(http/http_dispatch)).
:- use_module(library(broadcast)).
:- use_module(library(semweb/turtle)).
:- use_module(library(semweb/rdf_persistency)).
:- use_module(library(oslc_dispatch)).

:- setting(oslc_prolog_server:base_uri, atom, 'http://localhost:3020/oslc/', 'Base URI').

:- http_handler('/oslc/', dispatcher, [prefix]).

:- listen(settings(changed(oslc_prolog_server:base_uri, Old, New)), ( % listen on base URI setting changes
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
  once((
    member(protocol(Protocol), Request),
    member(host(Host), Request),
    member(port(Port), Request),
    member(path(Path), Request),
    atomic_list_concat([Protocol, '://', Host, ':', Port,  Path], '', Uri), % determine URI called
    setting(oslc_prolog_server:base_uri, BaseUri),
    atom_concat(BaseUri, ServicePath, Uri), % check if URI called starts with the base URI
    select_content_type(Request, ContentType), % compute content-type to serve based on request
    ( ContentType \== nil
    -> split_string(ServicePath, "/", "/", Parts),
       strings_to_atoms(Parts, [Prefix|ResourceSegments]),
       rdf_current_prefix(Prefix, _),
       setup_call_cleanup(make_graph(GraphOut), ( % create a new temporary RDF graph
         dispatch(Prefix:ResourceSegments, Request, _GraphIn, GraphOut),
         rdf_graph_property(GraphOut, triples(Triples)),
         ( Triples > 0
         -> format('Status: 200~n'),
            format('Content-type: ~w; charset=utf-8~n~n', [ContentType]),
            serializer(ContentType, Serializer), % select proper serializer
            current_output(Out),
            save_graph(Out, GraphOut, Serializer) % serialize temporary RDF graph to the response
         ; true
         )
       ), rdf_unload_graph(GraphOut)) % regardless of the result remove temporary RDF graph
    ; format('Status: 415~n~n') % requested content-type cannot be served
    )
  ; format('Status: 404~n~n') % requested resource not found
  )).

strings_to_atoms([], []) :- !.
strings_to_atoms([S|T], [A|T2]) :-
  atom_string(A, S),
  strings_to_atoms(T, T2).

make_graph(Graph) :-
  uuid(Graph), % generate unique identifier for the temporary RDF graph
  rdf_create_graph(Graph),
  rdf_persistency(Graph, false). % do not persist (keep only in RAM)

uuid(Id) :-
  Max is 1<<128,
  random_between(0, Max, Num),
  atom_number(Id, Num).

select_content_type(Request, ContentType) :-
  member(accept(Accept), Request), % fetch accept header
  predsort(accept_compare, Accept, AcceptSorted), % sort requested content types according to qualities
  select_content_type0(AcceptSorted, ContentType). % select the best matching content type

select_content_type0([], nil) :- !.
select_content_type0([media(H,_,_,_)|T], ContentType) :- % go through all requested content types
  ( serializer(H, _)
  -> !, ContentType = H % if content type is */* then H becomes _VAR/_VAR and matches application/'rdf+xml'
  ; select_content_type0(T, ContentType)
  ).

accept_compare(<, media(_,_,W1,_), media(_,_,W2,_)) :- % sort qualities in reverse order - biggest first
  W1 > W2.
accept_compare(<, _, _).

serializer(application/'rdf+xml', rdf).
serializer(text/'rdf+xml', rdf).
serializer(application/'x-turtle', turtle).
serializer(application/'turtle', turtle).
serializer(text/'turtle', turtle).

save_graph(Out, Graph, rdf) :-
  rdf_save(stream(Out), [graph(Graph)]).

save_graph(Out, Graph, turtle) :-
  rdf_save_turtle(Out, [graph(Graph), comment(false), silent(true), tab_distance(0)]).
