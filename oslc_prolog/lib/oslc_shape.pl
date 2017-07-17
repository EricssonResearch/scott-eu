:- module(oslc_shape, [
  check_occurs/4,
  check_value_type/4
]).

:- use_module(library(semweb/rdf_library)).
:- use_module(library(oslc_error)).

:- rdf_register_prefix(oslc, 'http://open-services.net/ns/core#').

:- rdf_meta format_value(r, -, -).
:- rdf_meta check_resource(r, -).
:- rdf_meta check_literal(r, -).

% ------------ CHECK PROPERTY OCCURS

check_occurs(IRI, PropertyResource, InternalValue, ReadValue) :-
  rdf(PropertyResource, oslc:occurs, Occurs),
  once((
    format_value(Occurs, InternalValue, ReadValue)
  ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
    oslc_error("Property ~w of resource ~w must have cardinality ~w", [PropertyDefinition, IRI, Occurs])
  )).

format_value(oslc:'Zero-or-one', [], V) :-
  var(V) ; V == [].
format_value(oslc:'Zero-or-many', [], V) :-
  var(V) ; V == [].
format_value(oslc:'Zero-or-one', [V], V) :-
  nonvar(V), \+ is_list(V).
format_value(oslc:'Exactly-one', [V], V) :-
  nonvar(V), \+ is_list(V).
format_value(oslc:'Zero-or-many', [V|T], [V|T]).
format_value(oslc:'One-or-many', [V|T], [V|T]).

% ------------ CHECK VALUE TYPE

check_value_type(IRI, PropertyResource, Value, Type) :-
  rdf(PropertyResource, oslc:valueType, Type),
  once((
    check_resource(Type, Value)
  ; check_literal(Type, Value)
  ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
    oslc_error("Property ~w of resource ~w must be of type ~w", [PropertyDefinition, IRI, Type])
  )).

% ------------ CHECK RESOURCE VALUE

check_resource(oslc:'LocalResource', Value) :-
  rdf_is_bnode(Value).

check_resource(oslc:'Resource', Value) :-
  \+ rdf_is_bnode(Value),
  rdf_is_resource(Value).

check_resource(oslc:'AnyResource', Value) :-
  rdf_is_resource(Value).

% ------------ CHECK LITERAL VALUE

check_literal(xsd:boolean, Value) :-
  member(Value, [true, false]).

check_literal(xsd:dateTime, Value) :-
  parse_time(Value, iso_8601, _).

check_literal(xsd:decimal, Value) :-
  float(Value).

check_literal(xsd:double, Value) :-
  float(Value).

check_literal(xsd:float, Value) :-
  float(Value).

check_literal(xsd:integer, Value) :-
  integer(Value).

check_literal(xsd:string, Value) :-
  atomic(Value).

check_literal(rdf:'XMLLiteral', Value) :-
  string(Value).
