:- module(oslc, [
  register_resource/1,
  oslc_resource/5
]).

:- use_module(library(semweb/rdf_library)).
:- use_module(library(oslc_shape)).
:- use_module(library(oslc_error)).

:- rdf_register_prefix(oslc, 'http://open-services.net/ns/core#').
:- rdf_register_prefix(oslcs, 'http://ontology.cf.ericsson.net/oslc_shapes#').
:- rdf_register_prefix(oslcp, 'http://ontology.cf.ericsson.net/oslc_prolog#').

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslc).
:- rdf_load_library(oslcs).
:- rdf_load_library(oslcp).

:- rdf_meta register_resource(r).
:- rdf_meta oslc_resource(r, r, t, -, -).

:- initialization init.

init :-
  forall(
    rdf(X, rdf:type, oslcp:'PrologResource'),
    register_resource(X)
  ).

% ------------ RESOURCE

register_resource(PrologResource) :-
  rdf(PrologResource, oslcp:prologModule, literal(type(xsd:string, ModuleName))),
  rdf(PrologResource, oslcp:prologPredicate, literal(type(xsd:string, PredicateName))),
  rdf(PrologResource, oslc:resourceShape, ResourceShape),
  P =.. [PredicateName, Spec, Options, Source, Sink],
  retractall(ModuleName:P),
  assertz((
    ModuleName:P :-
      rdf_global_id(Spec, IRI),
      oslc_resource(IRI, ResourceShape, Options, Source, Sink)
  )),
  ModuleName:export(PredicateName/4).

oslc_resource(IRI, ResourceShape, Options, Source, Sink) :-
  rdf_transaction(
    oslc_resource0(IRI, ResourceShape, Options, Source, Sink)
  ).

oslc_resource0(IRI, ResourceShape, Options, Source, Sink) :-
  rdf_global_id(oslcs:rdfType, TypePropertyResource),
  once((
    read_property(IRI, TypePropertyResource, A, Source),
    nonvar(A)
  ; findall(V, rdf(ResourceShape, oslc:describes, V), Resources),
    write_property(IRI, TypePropertyResource, Resources, Sink)
  )),
  findall(P, rdf(ResourceShape, oslc:property, P), Properties),
  oslc_properties(IRI, ResourceShape, Properties, Options, Source, Sink).

% ------------ PROPERTIES

oslc_properties(IRI, _, [], Options, _, _) :-
  once((
    length(Options, L), L == 0
  ; oslc_error("Unknown or duplicate properties ~w of resource ~w", [Options, IRI])
  )).

oslc_properties(IRI, ResourceShape, [PropertyResource|T], Options, Source, Sink) :-
  rdf(PropertyResource, oslc:name, literal(type(xsd:string, Property))),
  ( once((
      H =.. [Property, Value], selectchk(H, Options, RemainingOptions)
    ; H = (Property = Value), selectchk(H, Options, RemainingOptions)
    )) % TODO: error if duplicates of Property(...) found in RemainingOptions
  -> ( var(Value)
     -> read_property(IRI, PropertyResource, Value, Source)
     ; once((
         is_list(Value),
         write_property(IRI, PropertyResource, Value, Sink)
       ; write_property(IRI, PropertyResource, [Value], Sink)
       ))
     ),
     oslc_properties(IRI, ResourceShape, T, RemainingOptions, Source, Sink)
  ; read_property(IRI, PropertyResource, _, Source),
    oslc_properties(IRI, ResourceShape, T, Options, Source, Sink)
  ).

% ------------ READ

read_property(IRI, PropertyResource, Value, Source) :-
  rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
  read_resource_property(IRI, PropertyDefinition, Values, Source),
  check_property_occurs(IRI, PropertyResource, Values),
  read_value_list(IRI, PropertyResource, Values),
  rdf(PropertyResource, oslc:occurs, Occurs),
  rdf_global_id(LOccurs, Occurs),
  format_value(LOccurs, Values, Value).

format_value(_, [], _) :- !.
format_value(oslc:'Zero-or-one', [V], V) :- !.
format_value(oslc:'Exactly-one', [V], V) :- !.
format_value(_, V, V).

read_value_list(_, _, []).

read_value_list(IRI, PropertyResource, [H|T]) :-
  read_property_value(IRI, PropertyResource, H),
  read_value_list(IRI, PropertyResource, T).

read_property_value(IRI, PropertyResource, Value) :-
  once((
    check_resource_value(PropertyResource, Value, _)
  ; check_literal_value(PropertyResource, Value, _)
  ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
    findall(T, rdf(PropertyResource, oslc:valueType, T), Types),
    oslc_error("Property ~w of resource ~w is not one of ~w", [PropertyDefinition, IRI, Types])
  )).

% ------------ WRITE

write_property(IRI, PropertyResource, Value, Sink) :-
  rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
  ( is_list(Value)
  -> check_property_occurs(IRI, PropertyResource, Value),
     remove_resource_property(IRI, PropertyDefinition, Sink),
     write_list(IRI, PropertyResource, Value, Sink)
  ; once((
      check_resource_value(PropertyResource, Value, _),
      % TODO: add resource inlining if representation is oslc:'Inline'
      write_resource_property(IRI, PropertyDefinition, Value, Sink)
    ; check_literal_value(PropertyResource, Value, Type),
      write_literal_property(IRI, PropertyDefinition, Value, Type, Sink)
    ; rdf(PropertyResource, oslc:valueType, Type),
      oslc_error("Property ~w of resource ~w must be of type ~w", [PropertyDefinition, IRI, Type])
    ))
  ).

write_list(_, _, [], _).

write_list(IRI, PropertyResource, [H|T], Sink) :-
  write_property(IRI, PropertyResource, H, Sink),
  write_list(IRI, PropertyResource, T, Sink).

% ------------ RDF SOURCE / SINK

read_resource_property(IRI, PropertyDefinition, Value, rdf(Graph)) :-
  findall(V, (
    rdf(IRI, PropertyDefinition, X, Graph),
    once((
      rdf_is_literal(X),
      X = literal(type(_, V))
    ; V = X
    ))
  ), Value).

remove_resource_property(IRI, PropertyDefinition, rdf(Graph)) :-
  rdf_retractall(IRI, PropertyDefinition, _, Graph).

write_resource_property(IRI, PropertyDefinition, Value, rdf(Graph)) :-
  rdf_assert(IRI, PropertyDefinition, Value, Graph).

write_literal_property(IRI, PropertyDefinition, Value, Type, rdf(Graph)) :-
  rdf_assert(IRI, PropertyDefinition, literal(type(Type, Value)), Graph).
