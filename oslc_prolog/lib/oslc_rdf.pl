:- module(oslc_rdf, [
  make_temp_graph/1,
  clean_temp_graphs/0,
  autodetect_resource_graph/2,
  resource_md5/3,
  graph_md5/2
]).

:- use_module(library(semweb/rdf_db), [rdf_is_resource/1]).
:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdf_persistency)).
:- use_module(library(oslc_shape)).

:- rdf_meta autodetect_resource_graph(r, -).
:- rdf_meta resource_md5(r, -, -).

:- thread_local temp_graph/1.

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

oslc:delete_property(IRI, PropertyDefinition, rdf) :-
  must_be(atom, IRI),
  rdf_retractall(IRI, PropertyDefinition, _).

oslc:delete_property(IRI, PropertyDefinition, rdf(Graph)) :-
  must_be(atom, IRI),
  must_be(atom, Graph),
  rdf_retractall(IRI, PropertyDefinition, _, Graph).

%!  make_temp_graph(-Graph) is det.
%
%   Create a new non-persistent (RAM only) graph with unique name.

make_temp_graph(Graph) :-
  must_be(var, Graph),
  uuid(Graph),
  rdf_create_graph(Graph),
  rdf_persistency(Graph, false),
  assertz(temp_graph(Graph)).

%!  clean_temp_graphs is det.
%
%   Delete temporary graphs created with make_temp_graph/1 in this thread.

clean_temp_graphs :-
  forall(
    temp_graph(Graph),
    rdf_unload_graph(Graph)
  ),
  retractall(temp_graph(_)).

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
%   in more that one graph. Ignores graphs created using make_temp_graph/1.

autodetect_resource_graph(IRI, Graph) :-
  must_be(ground, IRI),
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
  must_be(ground, IRI),
  must_be(atom, Graph),
  rdf_global_id(IRI, Id),
  resource_content(Id, Graph, ContentList),
  flatten([Id, ContentList], FlatContentList),
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
  must_be(atom, Graph),
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
