:- module(oslc_rdf, []).

:- use_module(library(semweb/rdf11)).
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
