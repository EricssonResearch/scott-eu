:- module(oslc, [
  register_resource/1,
  oslc_resource/4
]).

:- use_module(library(semweb/rdf_library)).

:- rdf_register_prefix(oslc, 'http://open-services.net/ns/core#').
:- rdf_register_prefix(oslcs, 'http://ontology.cf.ericsson.net/oslc_shapes#').
:- rdf_register_prefix(oslcp, 'http://ontology.cf.ericsson.net/oslc_prolog#').

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslc).
:- rdf_load_library(oslcs).
:- rdf_load_library(oslcp).

:- rdf_meta register_resource(r).
:- rdf_meta oslc_resource(r, r, t, -).

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
  P =.. [PredicateName, Spec, Options, Graph],
  retractall(ModuleName:P),
  assertz((
    ModuleName:P :-
      rdf_global_id(Spec, IRI),
      oslc_resource(IRI, ResourceShape, Options, Graph)
  )),
  ModuleName:export(PredicateName/3).

oslc_resource(IRI, ResourceShape, Options, Graph) :-
  rdf_transaction((
    rdf(ResourceShape, oslc:describes, Resource),
    rdf_assert(IRI, rdf:type, Resource, Graph),
    findall(P, rdf(ResourceShape, oslc:property, P), Properties),
    oslc_properties(IRI, ResourceShape, Properties, Options, Graph)
  )).

% ------------ PROPERTIES

oslc_properties(IRI, _, [], Options, _) :-
  once((
    length(Options, L), L == 0
  ; error("Unknown or duplicate properties ~w of resource ~w", [Options, IRI])
  )).

oslc_properties(IRI, ResourceShape, [PropertyResource|T], Options, Graph) :-
  once((
    rdf(PropertyResource, oslcp:prologOption, literal(type(xsd:string, Property))),
    ( once((
        H =.. [Property, Value], selectchk(H, Options, RemainingOptions)
      ; H = (Property = Value), selectchk(H, Options, RemainingOptions)
      ))
    -> ( var(Value)
       -> % read property
          % TODO: call prologGetter if exists instead of read_property
          read_property(IRI, PropertyResource, Value, Graph)
       ; % write property
         rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
         rdf_retractall(IRI, PropertyDefinition, _, Graph),
         % TODO: call prologSetter if exists instead of write_property
         write_property(IRI, PropertyResource, Value, Graph)
       ),
       oslc_properties(IRI, ResourceShape, T, RemainingOptions, Graph)
    ; % check property
      % TODO: check if current database is consistent with shape
      oslc_properties(IRI, ResourceShape, T, Options, Graph)
    )
  ; error("Resource ~w does not have prolog option defined for property ~w", [IRI, PropertyResource])
  )).

% ------------ READ

read_property(IRI, PropertyResource, Value, Graph) :-
  get_occurs(PropertyResource, Occurs, ZO, ZM, OM, _),
  rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
  findall(V, rdf(IRI, PropertyDefinition, V, Graph), Values),
  length(Values, X),
  ( X == 0
  -> % Values is empty
     once((
       Occurs == ZO
     ; Occurs == ZM
     ; error("Required property ~w of resource ~w is missing", [PropertyDefinition, IRI])
     ))
  ; ( X > 1
    -> % Values should be a list
       ( ( Occurs == OM ; Occurs == ZM )
       -> read_list(IRI, PropertyResource, Values, Value)
       ; error("Property ~w of resource ~w occurred more than once", [PropertyDefinition, IRI])
       )
    ; ( % Values should be a list with a single element
        ( Occurs == OM ; Occurs == ZM )
      -> read_list(IRI, PropertyResource, Values, Value)
      ; % Values should not be a list
        [V] = Values,
        read_value(IRI, PropertyResource, V, Value)
      )
    )
  ).

read_list(_, _, [], []).
read_list(IRI, PropertyResource, [H|T], [H2|T2]) :-
  read_value(IRI, PropertyResource, H, H2),
  read_list(IRI, PropertyResource, T, T2).

read_value(IRI, PropertyResource, literal(type(Type, Value)), Value) :- !,
  once((
    rdf_global_id(LType, Type),
    check_literal(LType, Value)
  ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
    rdf(PropertyResource, oslc:valueType, T),
    error("Property ~w of resource ~w is not of literal type ~w", [PropertyDefinition, IRI, T])
  )).

read_value(IRI, PropertyResource, Value, Value) :-
  check_resource(IRI, PropertyResource, Value), !.

read_value(IRI, PropertyResource, _, _) :-
  rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
  rdf(PropertyResource, oslc:valueType, Type),
  error("Failed to read property ~w of resource ~w (should be of type ~w)", [PropertyDefinition, IRI, Type]).

% ------------ WRITE

% Value is an empty list
write_property(IRI, PropertyResource, [], _) :- !,
  ( rdf(PropertyResource, oslc:occurs, oslc:'Zero-or-many')
  -> true
  ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
    error("Property ~w of resource ~w cannot be set to empty list", [PropertyDefinition, IRI])
  ).

% Value is a list
write_property(IRI, PropertyResource, [H|T], Graph) :- !,
  ( once((
      rdf(PropertyResource, oslc:occurs, oslc:'Zero-or-many')
    ; rdf(PropertyResource, oslc:occurs, oslc:'One-or-many')
    ))
  -> write_list(IRI, PropertyResource, [H|T], Graph)
  ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
    error("Property ~w of resource ~w cannot be set to a list", [PropertyDefinition, IRI])
  ).

% Value is not a list
write_property(IRI, PropertyResource, Value, Graph) :-
  check_resource(IRI, PropertyResource, Value), !,
  once((
    Value == ''
  ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
    rdf_assert(IRI, PropertyDefinition, Value, Graph)
  )).

% Value is not a list and not a resource
write_property(IRI, PropertyResource, Value, Graph) :-
  ( ( rdf(PropertyResource, oslc:occurs, oslc:'Exactly-one'),
      Value == ''
    )
  -> rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
     error("Property ~w of resource ~w must occur exactly once", [PropertyDefinition, IRI])
  ; ( Value == ''
    -> true
    ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
      rdf(PropertyResource, oslc:valueType, Type),
      ( rdf_global_id(LType, Type),
        check_literal(LType, Value)
      -> rdf_assert(IRI, PropertyDefinition, literal(type(Type, Value)), Graph)
      ; error("Property ~w of resource ~w must be of literal type ~w", [PropertyDefinition, IRI, Type])
      )
    )
  ).

% ------------ CHECK RESOURCE

check_resource(IRI, PropertyResource, Value) :-
  rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
  rdf(PropertyResource, oslc:valueType, Type),
  rdf_global_id(oslc:'LocalResource', LR),
  rdf_global_id(oslc:'Resource', RE),
  rdf_global_id(oslc:'AnyResource', AR),
  member(Type, [LR, RE, AR]),
  % TODO: check resource range if given
  once((
    Type == LR,
    rdf_is_bnode(Value)
  ; ( Type == RE ; Type == AR ),
    rdf_is_resource(Value)
  ; error("Property ~w of resource ~w must be of type ~w", [PropertyDefinition, IRI, Type])
  )).

% ------------ CHECK LITERALS

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

% ------------ LIST

write_list(_, _, [], _).
write_list(IRI, PropertyResource, [H|T], Graph) :-
  write_property(IRI, PropertyResource, H, Graph),
  write_list(IRI, PropertyResource, T, Graph).

% ------------ CHECK SHAPE

check_shape(IRI, ResourceShape, Graph) :-
  forall(
    rdf(ResourceShape, oslc:property, PropertyResource),
    check_property(IRI, PropertyResource, Graph)
  ).

check_property(IRI, PropertyResource, Graph) :-
  get_occurs(PropertyResource, Occurs, ZO, ZM, OM, _),
  rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
  aggregate_all(count, rdf(IRI, PropertyDefinition, _, Graph), X),
  ( X == 0
  -> once((
       Occurs == ZO
     ; Occurs == ZM
     ; error("Property ~w of resource ~w must be specified", [PropertyDefinition, IRI])
     ))
  ; ( X > 1
    -> once((
         Occurs == OM
       ; Occurs == ZM
       ; error("Property ~w of resource ~w must not occur more than once", [PropertyDefinition, IRI])
       ))
    ; true
    )
  ).

% ------------ OCCURS

get_occurs(PropertyResource, Occurs, ZO, ZM, OM, EO) :-
  rdf(PropertyResource, oslc:occurs, Occurs),
  rdf_global_term([oslc:'Zero-or-one',
                   oslc:'Zero-or-many',
                   oslc:'One-or-many',
                   oslc:'Exactly-one'],
                  [ZO, ZM, OM, EO]),
  member(Occurs, [ZO, ZM, OM, EO]).

% ------------ ERROR

% FIXME: probably there is a better way to report errors...

error(Message, Arguments) :-
  error0(Message, Arguments, []).
error0(Message, [], Accum):-
  reverse(Accum, Arguments),
  format(atom(Error), Message, Arguments),
  throw(Error).
error0(Message, [H|T], Accum) :-
  ( rdf_is_resource(H)
  -> rdf_global_id(H2, H)
  ; H2 = H
  ),
  error0(Message, T, [H2|Accum]).


%iri_xml_namespace(PropertyProperty, NS, Local),
%rdf_current_prefix(Prefix, NS),
%format(atom(PPK), '~w_~w', [Prefix, Local]),

/*
r_property(URI, Key, Value, Graph) :-
  rdf(URI, Key, B, Graph),
  rdf_is_bnode(B),
  rdf(B, rdf:type, pl:'PredicateCall', Graph),
  rdf(B, pl:prologModule, Module, Graph),
  rdf(B, pl:prologTerm, Term, Graph),
  T =.. [Term, URI, Key, Value, Graph],
  call(Module:T).

w_property(URI, Key, call(Module:Term), Graph) :-
  rdf_create_bnode(B),
  rdf_assert(B, rdf:type, pl:'PredicateCall', Graph),
  rdf_assert(B, pl:prologModule, Module, Graph),
  rdf_assert(B, pl:prologTerm, Term, Graph),
  rdf_assert(URI, Key, B, Graph).
*/
