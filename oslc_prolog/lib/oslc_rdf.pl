:- module(oslc_rdf, [
  make_graph/1,
  autodetect_resource_graph/2,
  resource_md5/3,
  graph_md5/2
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdf_persistency)).
:- use_module(library(oslc_shape)).

:- rdf_meta oslc_LocalResource(r).
:- rdf_meta oslc_Resource(r).

oslc_LocalResource(oslc:'LocalResource').
oslc_Resource(oslc:'Resource').

% ------------ RDF SOURCE / SINK

oslc:marshal_property(IRI, PropertyDefinition, Value, Type, rdf(Graph)) :-
  must_be(atom, IRI),
  must_be(atom, PropertyDefinition),
  must_be(ground, Value),
  must_be(atom, Graph),
  ( nonvar(Type),
    is_literal_type(Type)
  -> rdf_assert(IRI, PropertyDefinition, Value^^Type, Graph)
  ; rdf_assert(IRI, PropertyDefinition, Value, Graph)
  ).

oslc:unmarshal_property(IRI, PropertyDefinition, Value, Type, rdf) :-
  rdf(IRI, PropertyDefinition, Object),
  unmarshal_type(Object, Value, Type).

oslc:unmarshal_property(IRI, PropertyDefinition, Value, Type, rdf(Graph)) :-
  must_be(atom, Graph),
  rdf(IRI, PropertyDefinition, Object, Graph),
  unmarshal_type(Object, Value, Type).

unmarshal_type(Value^^Type, Value, Type) :- !,
  is_literal_type(Type).

unmarshal_type(Value, Value, Type) :-
  ( rdf_is_bnode(Value)
  -> oslc_LocalResource(Type)
  ; rdf_is_resource(Value),
    oslc_Resource(Type)
  ).

oslc:delete_property(IRI, PropertyDefinition, rdf) :-
  must_be(atom, IRI),
  rdf_retractall(IRI, PropertyDefinition, _).

oslc:delete_property(IRI, PropertyDefinition, rdf(Graph)) :-
  must_be(atom, IRI),
  must_be(atom, Graph),
  rdf_retractall(IRI, PropertyDefinition, _, Graph).

%!  make_graph(-Graph) is det.
%
%   Create a new non-persistent (RAM only) graph with unique name.

make_graph(Graph) :-
  uuid(Graph),
  rdf_create_graph(Graph),
  rdf_persistency(Graph, false).

uuid_salt('$oslc_salt_').

uuid(Id) :-
  Max is 1<<128,
  random_between(0, Max, Num),
  uuid_salt(Salt),
  atom_concat(Salt, Num, Id).

%!  autodetect_resource_graph(+IRI, +Graph) is det.
%
%   Tries to automatically detect Graph, in which resource IRI is defined.
%   Fails if resource has =|rdf:type|= property in more than one graph,
%   or doesn't have =|rdf:type|= property but is referrred to as a subject
%   in more that one graph. Ignores graphs created using make_graph/2.

autodetect_resource_graph(IRI, Graph) :-
  uuid_salt(Salt),
  once((
    findall(G, (
      rdf(IRI, rdf:type, _, G),
      \+ atom_concat(Salt, _, G)
    ), Graphs),
    sort(Graphs, SortedGraphs),
    [Graph] = SortedGraphs
  ; findall(G, (
      rdf(IRI, _, _, G),
      \+ atom_concat(Salt, _, G)
    ), Graphs),
    sort(Graphs, SortedGraphs),
    [Graph] = SortedGraphs
  )).

%!  resource_md5(+IRI, +Graph, -Hash) is det.
%
%   Hash is MD5 hash of resource IRI in graph Graph. Names of
%   blank nodes do not affect the hash.

resource_md5(IRI, Graph, Hash) :-
  resource_content(IRI, Graph, ContentList),
  flatten([IRI, ContentList], FlatContentList),
  atomics_to_string(FlatContentList, String),
  md5_hash(String, Hash, []).

resource_content(IRI, Graph, ContentList) :-
  findall([Predicate, Value], (
    rdf(IRI, Predicate, Object, Graph),
    ( rdf_is_bnode(Object)
    -> resource_content(Object, Graph, Value)
    ; term_string(Object, Value)
    )
  ), ContentList).

%!  graph_md5(+Graph, -Hash) is det.
%
%   Hash is MD5 hash of content in graph Graph. Names of blank
%   nodes do not affect the hash. If Graph contains only one resource
%   =R=, its hash is equal to =|resource_md5(R, Graph, Hash)|=.

graph_md5(Graph, Hash) :-
  findall(Subject, (
    rdf(Subject, _, _, Graph),
    \+ rdf_is_bnode(Subject)
  ), Resources),
  sort(Resources, SortedResources),
  findall([Resource, R], (
    member(Resource, SortedResources),
    resource_content(Resource, Graph, R)
  ), ContentList),
  flatten(ContentList, FlatContentList),
  atomics_to_string(FlatContentList, String),
  md5_hash(String, Hash, []).
