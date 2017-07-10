:- module(oslc, [
  register_resource/2,
  oslc_resource/4,
  oslc_properties/4
]).

:- use_module(library(semweb/rdf_library)).

:- rdf_register_prefix(oslc, 'http://open-services.net/ns/core#').
:- rdf_register_prefix(oslcs, 'http://ontology.cf.ericsson.net/oslc_shapes#').
:- rdf_register_prefix(oslcp, 'http://ontology.cf.ericsson.net/oslc_prolog#').

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslc).
:- rdf_load_library(oslcs).
:- rdf_load_library(oslcp).

:- rdf_meta register_resource(r, -).
:- rdf_meta oslc_resource(r, r, t, -).
:- rdf_meta oslc_properties(r, r, t, -).

:- initialization init.

init :-
  forall(
    rdf(X, rdf:type, oslcp:'PrologPredicate', 'http://ontology.cf.ericsson.net/oslc_prolog'),
    register_resource(X, oslc)
  ).

register_resource(PredicateResource, Module) :-
  rdf(PredicateResource, oslcp:prologName, literal(type(xsd:string, PredicateName))),
  rdf(PredicateResource, oslc:resourceShape, ResourceShape),
  P =.. [PredicateName, Spec, Options, Graph],
  retractall(Module:P),
  assertz((
    Module:P :-
      rdf_global_id(Spec, IRI),
      oslc_resource(IRI, ResourceShape, Options, Graph)
  )),
  Module:export(PredicateName/3).

oslc_resource(IRI, ResourceShape, Options, Graph) :-
  rdf_transaction((
    rdf(ResourceShape, oslc:describes, Resource),
    rdf_assert(IRI, rdf:type, Resource, Graph),
    oslc_properties(IRI, ResourceShape, Options, Graph)
  )).

oslc_properties(_, _, [], _).
oslc_properties(IRI, ResourceShape, [H|T], Graph) :-
  once((
    H =.. [Property, Value]
  ; H = (Property=Value)
  )),
  rdf(ResourceShape, oslc:property, PropertyResource),
  rdf(PropertyResource, oslcp:prologName, literal(type(xsd:string, Property))),
  rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
  rdf(PropertyResource, oslc:valueType, Type),
  ( var(Value)
  -> rdf(IRI, PropertyDefinition, ReadValue, Graph),
     format_value(ReadValue, Type, Value)
  ;  format_value(WriteValue, Type, Value),
     rdf_assert(IRI, PropertyDefinition, WriteValue, Graph)
  ),
  oslc_properties(IRI, ResourceShape, T, Graph).

format_value(Value, 'http://www.w3.org/2000/01/rdf-schema#Resource', Value).

format_value(literal(type(Type, Value)), Type, Value).


%iri_xml_namespace(PropertyProperty, NS, Local),
%rdf_current_prefix(Prefix, NS),
%format(atom(PPK), '~w_~w', [Prefix, Local]),
