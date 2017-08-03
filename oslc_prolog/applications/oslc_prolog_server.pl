:- module(oslc_prolog_server, []).

:- use_module(library(http/thread_httpd)).
:- use_module(library(http/http_client)).
:- use_module(library(http/http_dispatch)).
:- use_module(library(broadcast)).
:- use_module(library(semweb/turtle)).
:- use_module(library(oslc_dispatch)).
:- use_module(library(oslc_rdf)).
:- use_module(library(oslc)).

:- setting(oslc_prolog_server:base_uri, atom, 'http://localhost:3020/', 'Base URI').
:- setting(oslc_prolog_server:exposed_prefixes, list(atom), [*], 'Exposed Prefixes').

:- http_handler('/', dispatcher, [prefix]).

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
  setup_call_cleanup(true, (
    catch((
      check_path(Request, Prefix, ResourceSegments),
      check_method(Request, Method),
      check_accept(Request, ContentType),
      ( member(Method, [post,put]) % if POST or PUT request and there is a body, read it
      -> read_request_body(Request, GraphIn)
      ; true
      ),
      once((
        dispatch(_{ request: Request,
                   iri_spec: Prefix:ResourceSegments,
                     method: Method,
               content_type: ContentType,
                   graph_in: GraphIn,
                  graph_out: GraphOut,
                    headers: Headers }), % main dispatch method
        ( ground(GraphOut),
          rdf_graph_property(GraphOut, triples(Triples)),
          Triples > 0 % the output document is not empty
        -> format_response_graph(200, GraphOut, Headers, ContentType)
        ; true % custom dispatcher should form full response by itself
        )
      ; throw(response(404)) % not found (failed to dispatch)
      ))
    ),
    E,
    ( E =.. [response|Args]
    -> FR =.. [format_error_response|[Request|Args]],
       call(FR)
    ; message_to_string(error(E, _), S),
      format_error_response(Request, 500, S) % internal server error
    ))
  ), clean_temp_graphs).

format_error_response(Request, StatusCode) :-
  format_error_response(Request, StatusCode, _, []).

format_error_response(Request, StatusCode, Message) :-
  format_error_response(Request, StatusCode, Message, []).

format_error_response(Request, StatusCode, Message, Headers) :-
  make_temp_graph(GraphErr),
  ( atomic(Message)
  -> ErrorMessage = Message
  ; once(error_message(StatusCode, ErrorMessage))
  ),
  create_resource('error', [oslc:'Error'], [oslc_shapes:errorShape], [statusCode = StatusCode, message = ErrorMessage], rdf(GraphErr)),
  catch(
    check_accept(Request, ContentType),
    _,
    serializer(ContentType, _)
  ),
  format_response_graph(StatusCode, GraphErr, Headers, ContentType).

error_message(400, 'Bad request').
error_message(404, 'Not found').
error_message(405, 'Method not allowed').
error_message(406, 'Not acceptable').
error_message(411, 'Content length required').
error_message(412, 'Precondition failed').
error_message(415, 'Unsupported media type').
error_message(_, 'No message').

format_response_graph(StatusCode, Graph, Headers, ContentType) :-
  must_be(integer, StatusCode),
  must_be(ground, Graph),
  must_be(ground, ContentType),
  format(atom(ContentTypeValue), '~w; charset=utf-8', [ContentType]),
  graph_md5(Graph, Hash),
  append(Headers, ['ETag'(Hash), 'Content-type'(ContentTypeValue)], NewHeaders),
  response(StatusCode, NewHeaders),
  serializer(ContentType, Serializer), % select proper serializer
  current_output(Out),
  save_graph(Out, Graph, Serializer). % serialize temporary RDF graph to the response

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
    rdf_current_prefix(Prefix, _),
    setting(oslc_prolog_server:exposed_prefixes, ExposedPrefixes),
    once((
      member(Prefix, ExposedPrefixes) % check if prefix is in the list of exposed prefixes
    ; member((*), ExposedPrefixes)    % ... or exposed list of prefixes contains wildcard (*)
    ))
  ; throw(response(404)) % not found
  )).

check_method(Request, Method) :-
  once((
    member(method(Method), Request),
    member(Method, [get,post,put,delete])
  ; throw(response(405)) % method not allowed
  )).

check_accept(Request, ContentType) :-
  once((
    member(accept(Accept), Request), % fetch accept header
    predsort(accept_compare, Accept, AcceptSorted), % sort requested content types according to qualities
    select_content_type(AcceptSorted, ContentType) % select the best matching content type
  ; throw(response(406)) % not acceptable
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
  ; throw(response(411)) % content length required
  )),
  once((
    member(content_type(InContentType), Request),
    once((
      serializer(Type, Format),
      format(atom(InContentType), '~w', Type)
    ))
  ; throw(response(415)) % unsupported media type
  )),
  http_read_data(Request, Data, [to(atom)]),
  open_string(Data, Stream),
  catch((
    make_temp_graph(GraphIn),
    rdf_load(stream(Stream), [graph(GraphIn), format(Format), silent(true), on_error(error), cache(false)])
  ),
    error(E, stream(Stream, Line, Column, _)),
  (
    message_to_string(error(E, _), S),
    format(atom(Message), 'Parsing error (line ~w, column ~w): ~w.', [Line, Column, S]),
    throw(response(400, Message)) % bad request
  )).

strings_to_atoms([], []) :- !.
strings_to_atoms([S|T], [A|T2]) :-
  atom_string(A, S),
  strings_to_atoms(T, T2).

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
