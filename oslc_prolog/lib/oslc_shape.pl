:- module(oslc_shape, [
  check_occurs/4,
  check_value_type/4,
  literal_types/1,
  is_literal_type/1,
  oslcInstanceShape/1
]).

:- use_module(library(semweb/rdf_library)).
:- use_module(library(oslc_error)).

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslc_shapes).

:- rdf_meta check_occurs(r, r, -, -).
:- rdf_meta format_value(r, -, -).
:- rdf_meta check_value_type(r, r, -, r).
:- rdf_meta check_value_type(r, -).
:- rdf_meta literal_types(t).
:- rdf_meta is_literal_type(r).

:- rdf_meta oslcInstanceShape(r).
oslcInstanceShape(oslc_shapes:oslcInstanceShape).

% ------------ CHECK PROPERTY OCCURS

check_occurs(IRI, PropertyResource, InternalValue, ReadValue) :-
  once((
    rdf(PropertyResource, oslc:occurs, Occurs),
  ( format_value(Occurs, InternalValue, ReadValue)
  ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
    oslc_error("Property [~w] of resource [~w] must have cardinality [~w]", [PropertyDefinition, IRI, Occurs])
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

check_value_type(_, _, [], _) :- !.

check_value_type(IRI, PropertyResource, [V|T], Type) :-
  once(rdf(PropertyResource, oslc:valueType, Type)),
  once((
    check_value_type(Type, V)
  ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
    oslc_error("Property [~w] of resource [~w] must be of type [~w]", [PropertyDefinition, IRI, Type])
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

% ------------ LITERAL TYPES

literal_types([ rdf:'XMLLiteral',
                xsd:string,
                xsd:integer,
                xsd:float,
                xsd:double,
                xsd:decimal,
                xsd:dateTime,
                xsd:boolean
              ]).

is_literal_type(Type) :-
  literal_types(LT),
  member(Type, LT).
