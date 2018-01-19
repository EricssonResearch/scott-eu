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

:- module(oslc_prolog_server, []).

:- use_module(library(semweb/rdf11)).
:- use_module(library(http/thread_httpd)).
:- use_module(library(http/http_client)).
:- use_module(library(http/http_dispatch)).
:- use_module(library(broadcast)).
:- use_module(library(oslc_dispatch)).
:- use_module(library(oslc_rdf)).
:- use_module(library(oslc)).

:- setting(oslc_prolog_server:prefix_path, atom, '/', 'Prefix for all OSLC paths').
:- setting(oslc_prolog_server:exposed_prefixes, list(atom), [*], 'Exposed prefixes').

:- listen(
     settings(changed(oslc_prolog_server:prefix_path, Old, New)), % listen on prefix path setting changes
     prefix_path_changed(Old, New)
   ).

:- cp_after_load(( % after ClioPatria has been completely loaded
     assertz(cp_after_loaded),
     setting(oslc_prolog_server:prefix_path, PrefixPath),
     prefix_path_changed(_, PrefixPath)
   )).

prefix_path_changed(OldPrefixPath, NewPrefixPath) :-
  ( current_predicate(cp_after_loaded/0) % if ClioPatria is completely loaded
  -> fix_slashes(OldPrefixPath, OldPath),
     fix_slashes(NewPrefixPath, NewPath),
     ( nonvar(OldPrefixPath)
     -> http_delete_handler(path(OldPath)) % remove old handler
     ; true
     ),
     ( NewPrefixPath == NewPath
     -> http_handler(NewPath, dispatcher, [prefix]) % set a handler for new prefix path
     ; set_setting(oslc_prolog_server:prefix_path, NewPath)
     )
  ; true
  ).

fix_slashes(InPath, OutPath) :-
  ( nonvar(InPath)
  -> ( sub_atom(InPath, 0, _, _, /)
     -> InPath2 = InPath
     ; atom_concat('/', InPath, InPath2)
     ),
     ( atom_concat(_, '/', InPath2)
     -> OutPath = InPath2
     ; atom_concat(InPath2, '/', OutPath)
     )
  ; true
  ).

dispatcher(Request) :-
  setup_call_cleanup(true, (
    catch(
      dispatcher0(Request),
      E,
      ( E =.. [response|Args]
      -> FR =.. [format_error_response|[Request|Args]],
         call(FR)
      ; message_to_string(error(E, _), S),
        format_error_response(Request, 500, S) % internal server error
      )
    )
  ), clean_temp_graphs).

dispatcher0(Request) :-
  check_path(Request, Prefix, ResourceSegments),
  check_method(Request, Method),
  check_accept(Request, ContentType),
  ( memberchk(Method, [post,put]) % if POST or PUT request and there is a body, read it
  -> read_request_body(Request, GraphIn)
  ; true
  ),
  ( memberchk(search(Search), Request),
    findall(Option, (
      member(Key=Value, Search),
      atom_concat('oslc.', OP, Key),
      Option =.. [OP, Value]
    ), Options)
  ; Options = []
  ),
  once((
    dispatch(_{ request: Request,
               iri_spec: Prefix:ResourceSegments,
                 method: Method,
           content_type: ContentType,
               graph_in: GraphIn,
              graph_out: GraphOut,
                headers: Headers,
                options: Options }), % main dispatch method
    ( ground(GraphOut),
      rdf_graph_property(GraphOut, triples(Triples)),
      Triples > 0 % the output document is not empty
    -> format_response_graph(200, GraphOut, Headers, ContentType)
    ; true % custom dispatcher should form full response by itself
    )
  ; throw(response(404)) % not found (failed to dispatch)
  )).

format_error_response(Request, StatusCode) :-
  format_error_response(Request, StatusCode, _, []).

format_error_response(Request, StatusCode, Message) :-
  format_error_response(Request, StatusCode, Message, []).

format_error_response(Request, StatusCode, Message, Headers) :-
  ( atomic(Message)
  -> ErrorMessage = Message
  ; once(oslc_dispatch:error_message(StatusCode, ErrorMessage))
  ),
  rdf_global_id(oslc_shapes:errorShape, ES),
  create_resource('error', [oslc:'Error'], [ES], [statusCode = StatusCode, message = ErrorMessage], tmp(GraphErr)),
  catch(
    check_accept(Request, ContentType),
    _,
    oslc_dispatch:serializer(ContentType, _)
  ),
  format_response_graph(StatusCode, GraphErr, Headers, ContentType).

format_response_graph(StatusCode, Graph, Headers, ContentType) :-
  must_be(integer, StatusCode),
  must_be(ground, Graph),
  must_be(ground, ContentType),
  format(atom(ContentTypeValue), '~w; charset=utf-8', [ContentType]),
  oslc_dispatch:serializer(ContentType, Serializer), % select proper serializer
  append(Headers, ['Content-type'(ContentTypeValue), 'Access-Control-Allow-Origin'('*')], InterimHeaders),
  ( memberchk(Serializer, [rdf, turtle])
  -> graph_md5(Graph, Hash),
     append(InterimHeaders, ['ETag'(Hash)], NewHeaders)
  ; NewHeaders = InterimHeaders
  ),
  response(StatusCode, NewHeaders),
  current_output(Out),
  oslc_dispatch:serialize_response(Out, Graph, Serializer). % serialize temporary RDF graph to the response

check_path(Request, Prefix, ResourceSegments) :-
  once((
    memberchk(path(Path), Request),
    setting(oslc_prolog_server:prefix_path, PrefixPath),
    atom_concat(PrefixPath, ServicePath, Path), % check if URI called starts with the prefix path
    split_string(ServicePath, "/", "/", Parts),
    strings_to_atoms(Parts, [Prefix|ResourceSegments]),
    rdf_current_prefix(Prefix, _),
    setting(oslc_prolog_server:exposed_prefixes, ExposedPrefixes),
    once((
      memberchk(Prefix, ExposedPrefixes) % check if prefix is in the list of exposed prefixes
    ; memberchk((*), ExposedPrefixes)    % ... or exposed list of prefixes contains wildcard (*)
    ))
  ; throw(response(404)) % not found
  )).

strings_to_atoms([], []) :- !.
strings_to_atoms([S|T], [A|T2]) :-
  atom_string(A, S),
  strings_to_atoms(T, T2).

check_method(Request, Method) :-
  once((
    memberchk(method(Method), Request),
    memberchk(Method, [get,post,put,delete])
  ; throw(response(405)) % method not allowed
  )).

check_accept(Request, ContentType) :-
  once((
    select_acceptable_content_type(Request, ContentType)
  ; throw(response(406)) % not acceptable
  )).

read_request_body(Request, GraphIn) :-
  once((
    memberchk(content_length(ContentLength), Request)
  ; throw(response(411)) % content length required
  )),
  ( ContentLength > 0
  -> once((
       memberchk(content_type(InContentType), Request),
       once((
         oslc_dispatch:serializer(Type, Format),
         format(atom(InContentType), '~w', Type)
       ))
     ; throw(response(415)) % unsupported media type
     )),
     ( memberchk(Format, [rdf, turtle])
     -> http_read_data(Request, Data, [to(atom)]),
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
        ))
     ; true
     )
  ; true
  ).
