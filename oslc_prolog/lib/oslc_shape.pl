:- module(oslc_shape, [
  check_property_occurs/3,
  check_resource_value/3,
  check_literal_value/3
]).

:- use_module(library(semweb/rdf_library)).
:- use_module(library(oslc_error)).

:- rdf_register_prefix(oslc, 'http://open-services.net/ns/core#').

% ------------ CHECK PROPERTY OCCURS

check_property_occurs(IRI, PropertyResource, Values) :-
  rdf(PropertyResource, oslc:occurs, Occurs),
  rdf_global_term([oslc:'Zero-or-one',
                   oslc:'Zero-or-many',
                   oslc:'One-or-many',
                   oslc:'Exactly-one'],
                  [ZO, ZM, OM, EO]),
  member(Occurs, [ZO, ZM, OM, EO]),
  length(Values, X),
  ( X == 0
  -> once((
       Occurs == ZO
     ; Occurs == ZM
     ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
       oslc_error("Mandatory property ~w of resource ~w is missing", [PropertyDefinition, IRI])
     ))
  ; ( X > 1
    -> once((
         Occurs == OM
       ; Occurs == ZM
       ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
         oslc_error("Property ~w of resource ~w must not occur more than once", [PropertyDefinition, IRI])
       ))
    ; true
    )
  ).

% ------------ CHECK RESOURCE VALUE

check_resource_value(PropertyResource, Value, Type) :-
  findall(T, rdf(PropertyResource, oslc:valueType, T), Types),
  member(Type, Types),
  rdf_global_id(LType, Type),
  check_resource(LType, Value).

check_resource(oslc:'LocalResource', Value) :-
  rdf_is_bnode(Value).

check_resource(oslc:'Resource', Value) :-
  \+ rdf_is_bnode(Value),
  rdf_is_resource(Value).

check_resource(oslc:'AnyResource', Value) :-
  rdf_is_resource(Value).

% ------------ CHECK LITERAL VALUE

check_literal_value(PropertyResource, Value, Type) :-
  findall(T, rdf(PropertyResource, oslc:valueType, T), Types),
  member(Type, Types),
  rdf_global_id(LType, Type),
  check_literal(LType, Value).

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
