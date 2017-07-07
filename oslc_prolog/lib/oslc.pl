:- module(oslc, [
  oslc_resource/5,
  oslc_property/5
]).

:- use_module(library(semweb/rdf_library)).

:- rdf_register_prefix(oslc, 'http://open-services.net/ns/core#').
:- rdf_register_prefix(eos, 'http://ontology.cf.ericsson.net/eos#').

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslc).
:- rdf_load_library(eos).

:- initialization init.

graph('http://ontology.cf.ericsson.net/eos').

init :-
  graph(Graph),
  forall(rdf(X, rdf:type, eos:'PrologPredicate', Graph), ignore(register_predicate(oslc, X))).

register_predicate(Module, Predicate) :-
  graph(Graph),
  rdf(Predicate, eos:prologName, literal(type(rdf:'XMLLiteral', Name)), Graph),

  P =.. [Name, Spec, POptions, PGraph],
  retractall(Module:P),

  rdf(Predicate, oslc:resourceShape, Shape, Graph),
  rdf(Shape, oslc:describes, PResource, Graph),

  findall(X, (
    rdf(Shape, oslc:property, Property, Graph),
    rdf(Property, eos:prologName, literal(type(rdf:'XMLLiteral', PropertyName)), Graph),
    findall(Y, (
      rdf(Property, PropertyProperty, PropertyValue, Graph),
      Y =.. [PropertyProperty, PropertyValue]
    ), PPs),
    dict_create(Z, _, PPs),
    X =.. [PropertyName, Z]
  ), Ps),

  dict_create(PDict, _, Ps),

  assertz((
    Module:P :-
      rdf_global_id(Spec, PIRI),
      oslc_resource(PIRI, PResource, PDict, POptions, PGraph)
  )).

oslc_resource(PIRI, PResource, PDict, POptions, PGraph) :-
  rdf_transaction((
    rdf_assert(PIRI, rdf:type, PResource, PGraph),
    oslc_property(PIRI, PResource, PDict, POptions, PGraph)
  )).

oslc_property(_, _, _, [], _) :- !.
oslc_property(PIRI, PResource, PDict, [H|T], PGraph) :-
  H =.. [Property, Value],
  ignore((
    PP = PDict.get(Property),
    rdf_global_id(oslc:propertyDefinition, OPD),
    ( var(Value)
    -> rdf(PIRI, PP.get(OPD), FormattedValue, PGraph),
      format_value(Value, _, FormattedValue)
    ; rdf_global_id(oslc:valueType, OVT),
      format_value(Value, PP.get(OVT), FormattedValue),
      rdf_assert(PIRI, PP.get(OPD), FormattedValue, PGraph)
    )
  )),
  oslc_property(PIRI, PResource, PDict, T, PGraph).

format_value(Value, Type, literal(type(Type, Value))) :-
  rdf_global_id(rdf:'XMLLiteral', T1),
  rdf_global_id(xsd:string, T2),
  member(Type, [T1, T2]), !.

format_value(Value, _, Value).

%iri_xml_namespace(PropertyProperty, NS, Local),
%rdf_current_prefix(Prefix, NS),
%format(atom(PPK), '~w_~w', [Prefix, Local]),
