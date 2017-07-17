:- module(oslc, [
  register_resource/1,
  oslc_resource/4,
  oslc_properties/4,
  oslc_remove_resource/2,
  oslc_remove_properties/3,
  oslc_copy_resource/4,
  oslc_copy_properties/5
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
:- rdf_meta oslc_properties(r, t, t, -).
:- rdf_meta oslc_remove_resource(r, -).
:- rdf_meta oslc_remove_properties(r, t, -).
:- rdf_meta oslc_copy_resource(r, r, -, -).
:- rdf_meta oslc_copy_properties(r, r, t, -, -).

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
    findall(R, rdf(ResourceShape, oslc:describes, R), ResourceList),
    sort(ResourceList, ResourceSet),
    once((
      read_property(IRI, TypePropertyResource, ITypes, _, SourceSink),
      sort(ITypes, ResourceSet)
    ; oslc_remove_properties0(IRI, [TypePropertyResource], SourceSink),
      write_property(IRI, TypePropertyResource, ResourceSet, _, SourceSink)
    )),
    findall(P, rdf(ResourceShape, oslc:property, P), Properties),
    oslc_properties0(IRI, Properties, Options, SourceSink)
  )).

% ------------ TRAVERSE PROPERTIES

oslc_properties(IRI, PropertyResources, Options, SourceSink) :-
  rdf_transaction((
    oslc_properties0(IRI, PropertyResources, Options, SourceSink)
  )).

oslc_properties0(_, [], _, _) :- !.

oslc_properties0(IRI, [PropertyResource|T], Options, SourceSink) :-
  once(rdf(PropertyResource, oslc:name, Property^^xsd:string)),
  atom_string(AProperty, Property),
  ( once((
      H =.. [AProperty, Value], selectchk(H, Options, RemainingOptions)
    ; H = (AProperty = Value), selectchk(H, Options, RemainingOptions)
    ))
  -> ( var(Value)
     -> read_property(IRI, PropertyResource, _, Value, SourceSink)
     ; oslc_remove_properties0(IRI, [PropertyResource], SourceSink),
       write_property(IRI, PropertyResource, _, Value, SourceSink)
     ),
     oslc_properties0(IRI, T, RemainingOptions, SourceSink)
  ; read_property(IRI, PropertyResource, _, _, SourceSink),
    oslc_properties0(IRI, T, Options, SourceSink)
  ).

% ------------ READ PROPERTY

read_property(IRI, PropertyResource, InternalValue, Value, Source) :-
  once(rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition)),
  unmarshal_property(IRI, PropertyDefinition, InternalValue, _, Source),
  once(rdf(PropertyResource, oslc:valueType, Type)),
  check_value_type(IRI, PropertyResource, InternalValue, Type),
  check_occurs(IRI, PropertyResource, InternalValue, Value).

% ------------ WRITE PROPERTY

write_property(IRI, PropertyResource, InternalValue, Value, Sink) :-
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
    read_property(IRI, TypePropertyResource, ITypes, _, SourceSink),
    forall((
      member(Type, ITypes),
      rdf(ResourceShape, rdf:type, oslc:'ResourceShape'),
      rdf(ResourceShape, oslc:describes, Type)
    ), (
      findall(P, rdf(ResourceShape, oslc:property, P), Properties),
      oslc_remove_properties0(IRI, Properties, SourceSink)
    )),
    once(rdf(TypePropertyResource, oslc:propertyDefinition, PropertyDefinition)),
    delete_property(IRI, PropertyDefinition, SourceSink)
  )).

% ------------ REMOVE PROPERTIES RECURSIVELY

oslc_remove_properties(IRI, PropertyResources, SourceSink) :-
  rdf_transaction((
    oslc_remove_properties0(IRI, PropertyResources, SourceSink)
  )).

oslc_remove_properties0(_, [], _) :- !.

oslc_remove_properties0(IRI, [PropertyResource|T], SourceSink) :-
  ( once(rdf(PropertyResource, oslc:valueType, oslc:'LocalResource'))
  -> read_property(IRI, PropertyResource, IValues, _, SourceSink),
     forall(
       member(Bnode, IValues),
       oslc_remove_resource(Bnode, SourceSink)
     )
  ; true
  ),
  once(rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition)),
  delete_property(IRI, PropertyDefinition, SourceSink),
  oslc_remove_properties0(IRI, T, SourceSink).

% ------------ COPY RESOURCE RECURSIVELY

oslc_copy_resource(IRIFrom, IRITo, Source, Sink) :-
  must_be(atom, IRIFrom),
  must_be(atom, IRITo),
  must_be(ground, Source),
  must_be(ground, Sink),
  rdf_transaction((
    oslcs_rdf_type(TypePropertyResource),
    oslc_copy_properties0(IRIFrom, IRITo, [TypePropertyResource], Source, Sink),
    read_property(IRIFrom, TypePropertyResource, ITypes, _, Source),
    forall((
      member(Type, ITypes),
      rdf(ResourceShape, rdf:type, oslc:'ResourceShape'),
      rdf(ResourceShape, oslc:describes, Type)
    ), (
      findall(P, rdf(ResourceShape, oslc:property, P), Properties),
      oslc_copy_properties0(IRIFrom, IRITo, Properties, Source, Sink)
    ))
  )).

% ------------ COPY PROPERTIES RECURSIVELY

oslc_copy_properties(IRIFrom, IRITo, PropertyResources, Source, Sink) :-
  rdf_transaction((
    oslc_copy_properties0(IRIFrom, IRITo, PropertyResources, Source, Sink)
  )).

oslc_copy_properties0(_, _, [], _, _) :- !.

oslc_copy_properties0(IRIFrom, IRITo, [PropertyResource|T], Source, Sink) :-
  read_property(IRIFrom, PropertyResource, IValues, Values, Source),
  ( ground(Values)
  -> oslc_remove_properties0(IRITo, [PropertyResource], Sink),
     ( once(rdf(PropertyResource, oslc:valueType, oslc:'LocalResource'))
     -> findall(Bnode, (
          member(Value, IValues),
          rdf_create_bnode(Bnode),
          oslc_copy_resource(Value, Bnode, Source, Sink)
        ), IResource),
        write_property(IRITo, PropertyResource, IResource, _, Sink)
     ; write_property(IRITo, PropertyResource, _, Values, Sink)
     )
  ; true
  ),
  oslc_copy_properties0(IRIFrom, IRITo, T, Source, Sink).
