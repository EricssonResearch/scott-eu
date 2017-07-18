:- module(oslc, [
  oslc_create_resource/4,
  oslc_resource/3,
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

:- rdf_meta oslc_create_resource(r, t, t, -).
:- rdf_meta oslc_resource(r, t, -).
:- rdf_meta oslc_remove_resource(r, -).
:- rdf_meta oslc_remove_properties(r, t, -).
:- rdf_meta oslc_copy_resource(r, r, -, -).
:- rdf_meta oslc_copy_properties(r, r, t, -, -).

:- rdf_meta rdf_type(r).
rdf_type(rdf:type).

oslc_create_resource(IRI, Types, Options, Sink) :-
  must_be(atom, IRI),
  must_be(list(atom), Types),
  must_be(list(ground), Options),
  must_be(ground, Sink),
  rdf_transaction((
    delete_property(IRI, _, Sink),
    rdf_type(RT),
    marshal_property(IRI, RT, Types, _, Sink),
    oslc_resource0(IRI, Options, Types, Sink)
  )).

%!  oslc_resource(:IRI, ?Options, ?SourceSink) is det.
%
%   Reads and/or writes properties of an OSLC resource IRI, from/to Source/Sink.
%   The shape of the OSLC resource must be provided in ResourceShape. The Options
%   is a list of properties of the resource in form of =|Key=Value|=.
%   The =Key= must correspond to the =oslc:name= property of the property's shape.
%   If =Value= is a variable, it will be unified with the value of the property =Key=
%   unmashalled from the SourceSink. If =Value= is grounded it will be set as a value of
%   =Key= and marshalled to the SourceSink. If =Value= is an empty list, property =Key=
%   will be removed from the SourceSink. It is allowed to combine read and write in a
%   single call, however, every property may appear in Options only once. For
%   triple store implementations of SourceSink use _rdf(Graph)_, where =Graph=
%   is a named graph in the triple store.

oslc_resource(IRI, Options, SourceSink) :-
  must_be(atom, IRI),
  must_be(list, Options),
  must_be(ground, SourceSink),
  rdf_transaction((
    rdf_type(RT),
    unmarshal_property(IRI, RT, Types, _, SourceSink),
    oslc_resource0(IRI, Options, Types, SourceSink)
  )).

oslc_resource0(_, [], _, _) :- !.

oslc_resource0(IRI, [Property=Value|T], Types, SourceSink) :-
  atom_string(Property, SProperty),
  once((
    member(Type, Types),
    % TODO: optimize this search
    rdf(ResourceShape, oslc:describes, Type),
    rdf(ResourceShape, oslc:property, PropertyResource),
    rdf(PropertyResource, oslc:name, SProperty^^xsd:string),
    ( var(Value)
    -> read_property(IRI, PropertyResource, _, Value, SourceSink)
    ; oslc_remove_properties0(IRI, [PropertyResource], SourceSink),
      write_property(IRI, PropertyResource, _, Value, SourceSink)
    )
  ) ; (
    ( var(Value)
    -> unmarshal_property(IRI, Property, Value, _, SourceSink)
    ; delete_property(IRI, Property, SourceSink),
      marshal_property(IRI, Property, Value, _, SourceSink)
    )
  )),
  oslc_resource0(IRI, T, Types, SourceSink).

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
    oslc_remove_resource0(IRI, SourceSink)
  )).

oslc_remove_resource0(IRI, SourceSink) :-
  rdf_type(RT),
  unmarshal_property(IRI, RT, Types, _, SourceSink),
  forall((
    member(Type, Types),
    rdf(ResourceShape, rdf:type, oslc:'ResourceShape'),
    rdf(ResourceShape, oslc:describes, Type)
  ), (
    findall(P, rdf(ResourceShape, oslc:property, P), Properties),
    oslc_remove_properties0(IRI, Properties, SourceSink)
  )),
  delete_property(IRI, RT, SourceSink).

% ------------ REMOVE PROPERTIES RECURSIVELY

oslc_remove_properties(IRI, PropertyResources, SourceSink) :-
  must_be(atom, IRI),
  must_be(ground, PropertyResources),
  must_be(ground, SourceSink),
  rdf_transaction((
    oslc_remove_properties0(IRI, PropertyResources, SourceSink)
  )).

oslc_remove_properties0(_, [], _) :- !.

oslc_remove_properties0(IRI, [PropertyResource|T], SourceSink) :-
  once(rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition)),
  ( once(rdf(PropertyResource, oslc:valueType, oslc:'LocalResource'))
  -> unmarshal_property(IRI, PropertyDefinition, IValues, _, SourceSink),
     forall(
       member(Bnode, IValues),
       oslc_remove_resource0(Bnode, SourceSink)
     )
  ; true
  ),
  delete_property(IRI, PropertyDefinition, SourceSink),
  oslc_remove_properties0(IRI, T, SourceSink).

% ------------ COPY RESOURCE RECURSIVELY

oslc_copy_resource(IRIFrom, IRITo, Source, Sink) :-
  must_be(atom, IRIFrom),
  must_be(atom, IRITo),
  must_be(ground, Source),
  must_be(ground, Sink),
  rdf_transaction((
    oslc_copy_resource0(IRIFrom, IRITo, Source, Sink)
  )).

oslc_copy_resource0(IRIFrom, IRITo, Source, Sink) :-
  rdf_type(RT),
  unmarshal_property(IRIFrom, RT, Types, _, Source),
  marshal_property(IRITo, RT, Types, _, Sink),
  forall((
    member(Type, Types),
    rdf(ResourceShape, rdf:type, oslc:'ResourceShape'),
    rdf(ResourceShape, oslc:describes, Type)
  ), (
    findall(P, rdf(ResourceShape, oslc:property, P), Properties),
    oslc_copy_properties0(IRIFrom, IRITo, Properties, Source, Sink)
  )).

% ------------ COPY PROPERTIES RECURSIVELY

oslc_copy_properties(IRIFrom, IRITo, PropertyResources, Source, Sink) :-
  must_be(atom, IRIFrom),
  must_be(atom, IRITo),
  must_be(ground, PropertyResources),
  must_be(ground, Source),
  must_be(ground, Sink),
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
          oslc_copy_resource0(Value, Bnode, Source, Sink)
        ), IResource),
        write_property(IRITo, PropertyResource, IResource, _, Sink)
     ; write_property(IRITo, PropertyResource, _, Values, Sink)
     )
  ; true
  ),
  oslc_copy_properties0(IRIFrom, IRITo, T, Source, Sink).
