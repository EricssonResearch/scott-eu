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

:- module(oslc_core, [
  post_resource/3
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdfs)).
:- use_module(library(oslc)).
:- use_module(library(oslc_rdf)).
:- use_module(library(oslc_dispatch)).
:- use_module(library(oslc_error)).

:- oslc_get((*):'', handle_ontology, 1).
:- oslc_get((*):(*), handle_get, 0).
:- oslc_post((*):(*), handle_post, 0).
:- oslc_put((*):(*), handle_put, 0).
:- oslc_delete((*):(*), handle_delete, 0).

:- oslc_get(oslc:time, get_current_time).
:- oslc_get(oslc:'Time', get_time_class).

get_current_time(Context) :-
  get_time(T),
  create_resource(oslc:time, [oslc:'Time'],
                 [dcterms:created='^^'(T, xsd:dateTime)], tmp(Context.graph_out)).

get_time_class(Context) :-
  create_resource(oslc:'Time', [rdfs:'Class'], [], tmp(Context.graph_out)).

handle_ontology(Context) :-
  findall(Graph, (
    rdf_graph(Graph),
    once((
      atom_concat(Graph, '/', Context.iri)
    ; atom_concat(Graph, '#', Context.iri)
    ; iri_xml_namespace(Graph, Context.iri, _)
    ))
  ), Graphs),
  output_graph(Graphs, Context).

output_graph([], _) :- !, fail.

output_graph([One], Context) :- !,
  Context.graph_out = One.

output_graph(Many, Context) :-
  GraphOut = Context.graph_out,
  make_temp_graph(GraphOut),
  forall(
    member(Graph, Many),
    forall(
      rdf(S, P, O, Graph),
      rdf_assert(S, P, O, GraphOut)
    )
  ).

handle_get(Context) :-
  IRI = Context.iri,
  once(rdf(IRI, _, _)),
  once((
    memberchk(search(Search), Context.request),
    L1 = [],
    ( memberchk(rdfs=sup, Search)
    -> findall(SuperClass, (
         rdfs_subclass_of(IRI, SuperClass),
         IRI \= SuperClass
       ), SuperClasses),
       append(L1, SuperClasses, L2)
    ; L2 = L1
    ),
    ( memberchk(rdfs=sub, Search)
    -> findall(SubClass, (
         rdfs_subclass_of(SubClass, IRI),
         IRI \= SubClass
       ), SubClasses),
       append(L2, SubClasses, L3)
    ; L3 = L2
    ),
    ( memberchk(rdfs=ind, Search)
    -> findall(Individual, (
         rdfs_individual_of(Individual, IRI),
         IRI \= Individual
       ), Individuals),
       append(L3, Individuals, L4)
    ; L4 = L3
    ),
    ( memberchk(rdfs=cls, Search)
    -> findall(Class, (
         rdfs_individual_of(IRI, Class),
         IRI \= Class
       ), Classes),
       append(L4, Classes, L5)
    ; L5 = L4
    ),
    list_to_set(L5, Set)
  ; Set = []
  )),
  catch((
    copy_resource([IRI|Set], [IRI|Set], rdf, tmp(Context.graph_out), [inline(rdf)|Context.options])
  ),
    oslc_error(Message),
    throw(response(400, Message)) % bad request (problem with Options)
  ).

handle_post(Context) :-
  post_resource(Context.iri, rdf(Context.graph_in), rdf(user)).

post_resource(IRI, Source, Sink) :-
  catch(
    post_resource0(IRI, Source, Sink),
    oslc_error(Message),
    throw(response(400, Message)) % bad request
  ).

post_resource0(IRI, Source, Sink) :-
  once((
    rdf(IRI, rdf:type, oslc:'CreationFactory')
  ; oslc_error('Resource [~w] is not a creation factory', [IRI])
  )),
  once((
    ground(Source)
  ; oslc_error('Missing resource in POST request to [~w]', [IRI])
  )),
  once((
    unmarshal_property(NewResource, rdf:type, Class, _, Source),
    \+ rdf_is_bnode(NewResource)
  ; oslc_error('Missing resource type in POST request to [~w]', [IRI])
  )),
  once((
    \+ unmarshal_property(NewResource, _, _, _, Sink)
  ; oslc_error('Resource [~w] already exists', [NewResource])
  )),
  once((
    unmarshal_list_property(IRI, oslc:resourceShape, Shapes, _, rdf),
    member(Shape, Shapes),
    rdf(Shape, rdf:type, oslc:'ResourceShape'),
    unmarshal_list_property(Shape, oslc:describes, Classes, _, rdf),
    member(Class, Classes)
  ; oslc_error('Creation factory [~w] does not support type [~w]', [IRI, Class])
  )),
  copy_resource(NewResource, NewResource, Source, Sink, []),
  setting(oslc_prolog_server:base_uri, BaseUri),
  uri_components(BaseUri, uri_components(C1, C2, PrefixPath, C4, C5)),
  split_string(PrefixPath, "/", "/", Parts),
  rdf_global_id(Prefix:Name, NewResource),
  append(Parts, [Prefix, Name], NewParts),
  atomics_to_string(NewParts, '/', NewPath),
  uri_components(Location, uri_components(C1, C2, NewPath, C4, C5)),
  response(201, ['Location'(Location)]). % created

handle_put(Context) :-
  catch(
    handle_put0(Context),
    oslc_error(Message),
    throw(response(400, Message)) % bad request
  ).

handle_put0(Context) :-
  once((
    memberchk(if_match(IfMatch), Context.request),
    atomic_list_concat([_, ReceivedHash, _], '\"', IfMatch)
  ; oslc_error('Missing or wrong header [If-Match] in PUT request to [~w]', [Context.iri_spec])
  )),
  IRI = Context.iri,
  autodetect_resource_graph(IRI, Graph),
  once((
    resource_md5(IRI, Graph, ReceivedHash),
    copy_resource(IRI, IRI, rdf(Context.graph_in), rdf(Graph), []),
    response(204) % no content
  ; format(atom(Message), 'The value of [If-Match] header does not match [~w]', [Context.iri_spec]),
    throw(response(412, Message)) % precondition failed
  )).

handle_delete(Context) :-
  catch((
    autodetect_resource_graph(Context.iri, Graph),
    delete_resource(Context.iri, rdf(Graph)),
    response(204) % no content
  ),
    oslc_error(Message),
    throw(response(400, Message)) % bad request
  ).
