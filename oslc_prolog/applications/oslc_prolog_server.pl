:- module(oslc_prolog_server, []).

:- use_module(library(http/thread_httpd)).
:- use_module(library(http/http_client)).
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
  catch(
    do_dispatch(Request),
  E, (
    ( E = status_code(StatusCode)
    -> format('Status: ~w~n~n', [StatusCode])
    ; format('Status: 500~n~n~w', [E]) % internal server error
    )
  )).

do_dispatch(Request) :-
  check_path(Request, Prefix, ResourceSegments),
  check_method(Request, Method),
  check_accept(Request, ContentType),
  setup_call_cleanup((
    make_graph(GraphIn),
    make_graph(GraphOut)
  ),(
    ( member(Method, [post,put]) % if POST or PUT request and there is a body, read it
    -> read_request_body(Request, GraphIn)
    ; true
    ),
    generate_response(Request, Prefix, ResourceSegments, ContentType, GraphIn, GraphOut)
  ),(
    rdf_unload_graph(GraphOut),
    rdf_unload_graph(GraphIn)
  )).

check_path(Request, Prefix, ResourceSegments) :-
  once((
    member(protocol(Protocol), Request),
    member(host(Host), Request),
    member(port(Port), Request),
    member(path(Path), Request),
    atomic_list_concat([Protocol, '://', Host, ':', Port,  Path], '', Uri), % determine URI called
    setting(oslc_prolog_server:base_uri, BaseUri),
    atom_concat(BaseUri, ServicePath, Uri), % check if URI called starts with the base URI
    split_string(ServicePath, "/", "/", Parts),
    strings_to_atoms(Parts, [Prefix|ResourceSegments]),
    rdf_current_prefix(Prefix, _)
  ; throw(status_code(404)) % not found
  )).

check_method(Request, Method) :-
  once((
    member(method(Method), Request),
    member(Method, [get,post,put,delete])
  ; throw(status_code(405)) % method not allowed
  )).

check_accept(Request, ContentType) :-
  once((
    member(accept(Accept), Request), % fetch accept header
    predsort(accept_compare, Accept, AcceptSorted), % sort requested content types according to qualities
    select_content_type(AcceptSorted, ContentType) % select the best matching content type
  ; throw(status_code(406)) % not acceptable
  )).

select_content_type([], _) :- fail.
select_content_type([media(H,_,_,_)|T], ContentType) :- % go through all requested content types
  ( serializer(H, _)
  -> !, ContentType = H % if content type is */* then H becomes _VAR/_VAR and matches application/'rdf+xml'
  ; select_content_type(T, ContentType)
  ).

read_request_body(Request, GraphIn) :-
  once((
    member(content_length(ContentLength), Request),
    ContentLength > 0
  ; throw(status_code(411)) % content length required
  )),
  once((
    member(content_type(InContentType), Request),
    once((
      serializer(Type, Format),
      format(atom(InContentType), '~w', Type)
    ))
  ; throw(status_code(415)) % unsupported media type
  )),
  http_read_data(Request, Data, [to(atom)]),
  open_string(Data, Stream),
  rdf_load(stream(Stream), [graph(GraphIn), format(Format), silent(true), cache(false)]).

generate_response(Request, Prefix, ResourceSegments, ContentType, GraphIn, GraphOut) :-
  once((
    dispatch(Prefix:ResourceSegments, Request, GraphIn, GraphOut) % main dispatch method
  ; throw(status_code(404)) % not found (failed to dispatch)
  )),
  rdf_graph_property(GraphOut, triples(Triples)),
  ( Triples > 0 % the output document is not empty
  -> format('Status: 200~n'),
    format('Content-type: ~w; charset=utf-8~n~n', [ContentType]),
    serializer(ContentType, Serializer), % select proper serializer
    current_output(Out),
    save_graph(Out, GraphOut, Serializer) % serialize temporary RDF graph to the response
  ; true % dispather should form full response by itself
  ).

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
  atom_concat('$oslc_salt_', Num, Id).

accept_compare(<, media(_,_,W1,_), media(_,_,W2,_)) :- % sort qualities in reverse order - biggest first
  W1 >= W2.
accept_compare(>, media(_,_,W1,_), media(_,_,W2,_)) :-
  W1 < W2.

serializer(application/'rdf+xml', rdf).
serializer(application/'turtle', turtle).
serializer(text/'rdf+xml', rdf).
serializer(application/'x-turtle', turtle).
serializer(text/'turtle', turtle).

save_graph(Out, Graph, rdf) :-
  rdf_save(stream(Out), [graph(Graph)]).

save_graph(Out, Graph, turtle) :-
  rdf_save_turtle(Out, [graph(Graph), comment(false), silent(true), tab_distance(0)]).
