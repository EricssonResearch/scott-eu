:- module(oslc, [
  register_resource/1,
  oslc_resource/4,
  oslc_remove_resource/2,
  oslc_copy_resource/4
]).

:- multifile marshal_property/5.
:- multifile unmarshal_property/5.
:- multifile delete_property/3.

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdf_library)).
:- use_module(library(oslc_shape)).
:- use_module(library(oslc_rdf)).
:- use_module(library(oslc_error)).

:- rdf_register_prefix(oslc, 'http://open-services.net/ns/core#').
:- rdf_register_prefix(oslcs, 'http://ontology.cf.ericsson.net/oslc_shapes#').
:- rdf_register_prefix(oslcp, 'http://ontology.cf.ericsson.net/oslc_prolog#').

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslc).
:- rdf_load_library(oslcs).
:- rdf_load_library(oslcp).

:- rdf_meta register_resource(r).
:- rdf_meta oslc_resource(r, r, t, -).
:- rdf_meta oslc_remove_resource(r, -).
:- rdf_meta oslc_copy_resource(r, r, -, -).

:- rdf_meta oslcs_rdf_type(r).
oslcs_rdf_type(oslcs:rdfType).

:- initialization init.

init :-
  forall(
    rdf(X, rdf:type, oslcp:'PrologResource'),
    register_resource(X)
  ).

%!  register_resource(:PrologResource) is det.
%
%   Creates and exports a predicate to operate PrologResource. The argument must
%   point to a resource of type =|oslcp:PrologResource|=. The created predicate will
%   have signature =|module:pred_name(IRI, Options, SourceSink)|=. Where =module=
%   is Prolog module (=prologModule= property of PrologResource), =pred_name= is the
%   predicate name of arity 3 (=prologPredicate= property of PrologResource). See
%   oslc_resource/4 for explanation of the arguments.

register_resource(PrologResource) :-
  must_be(atom, PrologResource),
  once((
    rdf(PrologResource, oslcp:prologModule, ModuleName^^xsd:string),
    rdf(PrologResource, oslcp:prologPredicate, PredicateName^^xsd:string),
    rdf(PrologResource, oslc:resourceShape, ResourceShape)
  )),
  atom_string(AModuleName, ModuleName),
  atom_string(APredicateName, PredicateName),
  P =.. [APredicateName, Spec, Options, SourceSink],
  retractall(AModuleName:P),
  assertz((
    AModuleName:P :-
      rdf_global_id(Spec, IRI),
      oslc_resource(IRI, ResourceShape, Options, SourceSink)
  )),
  AModuleName:export(APredicateName/3).

%!  oslc_resource(:IRI, :ResourceShape, ?Options, ?SourceSink) is det.
%
%   Reads and/or writes properties of an OSLC resource IRI, from/to Source/Sink.
%   The shape of the OSLC resource must be provided in ResourceShape. The Options
%   is a list of properties of the resource in form of =|Key(Value)|= or =|Key=Value|=.
%   The =Key= must correspond to the =oslc:name= property of the property's shape.
%   If =Value= is a variable, it will be unified with the value of the property =Key=
%   unmashalled from the SourceSink. If =Value= is grounded it will be set as a value of
%   =Key= and marshalled to the SourceSink. If =Value= is an empty list, property =Key=
%   will be removed from the SourceSink. It is allowed to combine read and write in a
%   single call, however, every property may appear in Options only once. For
%   triple store implementations of SourceSink use _rdf(Graph)_, where =Graph=
%   is a named graph in the triple store.

oslc_resource(IRI, ResourceShape, Options, SourceSink) :-
  must_be(atom, IRI),
  must_be(atom, ResourceShape),
  rdf_transaction((
    oslcs_rdf_type(TypePropertyResource),
    once((
      read_property(IRI, TypePropertyResource, Resources, SourceSink),
      nonvar(Resources)
      % TODO: check if Resources is the list of correct resources
    ; findall(R, rdf(ResourceShape, oslc:describes, R), Resources),
      write_property(IRI, TypePropertyResource, Resources, SourceSink)
    )),
    findall(P, rdf(ResourceShape, oslc:property, P), Properties),
    oslc_properties(IRI, Properties, Options, SourceSink)
  )).

% ------------ TRAVERSE PROPERTIES

oslc_properties(_, [], _, _).

oslc_properties(IRI, [PropertyResource|T], Options, SourceSink) :-
  once(rdf(PropertyResource, oslc:name, Property^^xsd:string)),
  atom_string(AProperty, Property),
  ( once((
      H =.. [AProperty, Value], selectchk(H, Options, RemainingOptions)
    ; H = (AProperty = Value), selectchk(H, Options, RemainingOptions)
    ))
  -> ( var(Value)
     -> read_property(IRI, PropertyResource, Value, SourceSink)
     ; oslc_remove_properties(IRI, [PropertyResource], SourceSink),
       write_property(IRI, PropertyResource, Value, SourceSink)
     ),
     oslc_properties(IRI, T, RemainingOptions, SourceSink)
  ; read_property(IRI, PropertyResource, _, SourceSink),
    oslc_properties(IRI, T, Options, SourceSink)
  ).

% ------------ READ PROPERTY

read_property(IRI, PropertyResource, Value, Source) :-
  once(rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition)),
  unmarshal_property(IRI, PropertyDefinition, InternalValue, _, Source),
  once(rdf(PropertyResource, oslc:valueType, Type)),
  check_value_type(IRI, PropertyResource, InternalValue, Type),
  check_occurs(IRI, PropertyResource, InternalValue, Value).

% ------------ WRITE PROPERTY

write_property(IRI, PropertyResource, Value, Sink) :-
  check_occurs(IRI, PropertyResource, InternalValue, Value),
  once(rdf(PropertyResource, oslc:valueType, Type)),
  check_value_type(IRI, PropertyResource, InternalValue, Type),
  once(rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition)),
  marshal_property(IRI, PropertyDefinition, InternalValue, Type, Sink).

% ------------ REMOVE RESOURCE RECURSIVELY

oslc_remove_resource(IRI, SourceSink) :-
  must_be(atom, IRI),
  must_be(ground, SourceSink),
  rdf_transaction((
    oslcs_rdf_type(TypePropertyResource),
    read_property(IRI, TypePropertyResource, ReadTypes, SourceSink),
    check_occurs(IRI, TypePropertyResource, Types, ReadTypes),
    forall((
      member(Type, Types),
      rdf(ResourceShape, rdf:type, oslc:'ResourceShape'),
      rdf(ResourceShape, oslc:describes, Type)
    ), (
      findall(P, rdf(ResourceShape, oslc:property, P), Properties),
      oslc_remove_properties(IRI, Properties, SourceSink)
    )),
    once(rdf(TypePropertyResource, oslc:propertyDefinition, PropertyDefinition)),
    delete_property(IRI, PropertyDefinition, SourceSink)
  )).

% ------------ REMOVE PROPERTIES RECURSIVELY

oslc_remove_properties(_, [], _).

oslc_remove_properties(IRI, [PropertyResource|T], SourceSink) :-
  ( once(rdf(PropertyResource, oslc:valueType, oslc:'LocalResource'))
  -> read_property(IRI, PropertyResource, ReadValues, SourceSink),
     ( ground(ReadValues)
     -> check_occurs(IRI, PropertyResource, Values, ReadValues),
        forall(
          member(Bnode, Values),
          oslc_remove_resource(Bnode, SourceSink)
        )
     ; true
     )
  ; true
  ),
  once(rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition)),
  delete_property(IRI, PropertyDefinition, SourceSink),
  oslc_remove_properties(IRI, T, SourceSink).

% ------------ COPY OSCL RESOURCE RECURSIVELY

oslc_copy_resource(IRIFrom, IRITo, Source, Sink) :-
  must_be(atom, IRIFrom),
  must_be(atom, IRITo),
  must_be(ground, Source),
  must_be(ground, Sink),
  rdf_transaction((
    oslcs_rdf_type(TypePropertyResource),
    oslc_copy_properties(IRIFrom, IRITo, [TypePropertyResource], Source, Sink),
    read_property(IRIFrom, TypePropertyResource, ReadTypes, Source),
    check_occurs(IRIFrom, TypePropertyResource, Types, ReadTypes),
    forall((
      member(Type, Types),
      rdf(ResourceShape, rdf:type, oslc:'ResourceShape'),
      rdf(ResourceShape, oslc:describes, Type)
    ), (
      findall(P, rdf(ResourceShape, oslc:property, P), Properties),
      oslc_copy_properties(IRIFrom, IRITo, Properties, Source, Sink)
    ))
  )).

% ------------ COPY PROPERTIES RECURSIVELY

oslc_copy_properties(_, _, [], _, _).

oslc_copy_properties(IRIFrom, IRITo, [PropertyResource|T], Source, Sink) :-
  read_property(IRIFrom, PropertyResource, ReadValues, Source),
  ( ground(ReadValues)
  -> oslc_remove_properties(IRITo, [PropertyResource], Sink),
     ( once(rdf(PropertyResource, oslc:valueType, oslc:'LocalResource'))
     -> check_occurs(IRIFrom, PropertyResource, Values, ReadValues),
        findall(Bnode, (
          member(Value, Values),
          rdf_create_bnode(Bnode),
          oslc_copy_resource(Value, Bnode, Source, Sink)
        ), InternalResource),
        check_occurs(IRITo, PropertyResource, InternalResource, Resource),
        write_property(IRITo, PropertyResource, Resource, Sink)
     ; write_property(IRITo, PropertyResource, ReadValues, Sink)
     )
  ; true
  ),
  oslc_copy_properties(IRIFrom, IRITo, T, Source, Sink).
