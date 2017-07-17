:- module(oslc_shape, [
  check_occurs/4,
  check_value_type/4
]).

:- use_module(library(semweb/rdf_library)).
:- use_module(library(oslc_error)).

:- rdf_register_prefix(oslc, 'http://open-services.net/ns/core#').

:- rdf_meta check_occurs(r, r, -, -).
:- rdf_meta format_value(r, -, -).
:- rdf_meta check_value_type(r, r, -, r).
:- rdf_meta check_value_type(r, -).

% ------------ CHECK PROPERTY OCCURS

check_occurs(IRI, PropertyResource, InternalValue, ReadValue) :-
  once((
    rdf(PropertyResource, oslc:occurs, Occurs),
  ( format_value(Occurs, InternalValue, ReadValue)
  ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
    oslc_error("Property ~w of resource ~w must have cardinality ~w", [PropertyDefinition, IRI, Occurs])
  ))).

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

check_value_type(_, _, [], _).

check_value_type(IRI, PropertyResource, [V|T], Type) :-
  once((
    check_value_type(Type, V)
  ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
    oslc_error("Property ~w of resource ~w must be of type ~w", [PropertyDefinition, IRI, Type])
  )),
  check_value_type(IRI, PropertyResource, T, Type).

check_value_type(oslc:'LocalResource', Value) :-
  rdf_is_bnode(Value).

check_value_type(oslc:'Resource', Value) :-
  \+ rdf_is_bnode(Value),
  rdf_is_resource(Value).

check_value_type(oslc:'AnyResource', Value) :-
  rdf_is_resource(Value).

check_value_type(xsd:boolean, Value) :-
  member(Value, [true, false]).

check_value_type(xsd:dateTime, Value) :-
  parse_time(Value, iso_8601, _).

check_value_type(xsd:decimal, Value) :-
  float(Value).

check_value_type(xsd:double, Value) :-
  float(Value).

check_value_type(xsd:float, Value) :-
  float(Value).

check_value_type(xsd:integer, Value) :-
  integer(Value).

check_value_type(xsd:string, Value) :-
  atomic(Value).

check_value_type(rdf:'XMLLiteral', Value) :-
  string(Value).
