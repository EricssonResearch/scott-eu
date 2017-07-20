:- module(oslc, [
  create_resource/4,
  applicable_shapes/2,
  create_resource/5,
  oslc_resource/3,
  applicable_shapes/3,
  copy_resource/5,
  delete_resource/2
]).

%!  marshal_property(+IRI, +PropertyDefinition, +Value, +Type, +Sink) is det.
%
%   Interface to define a procedure of writing Value and Type of property
%   PropertyDefinition of resource IRI to Sink.

:- multifile marshal_property/5.

%!  unmarshal_property(+IRI, +PropertyDefinition, -Value, -Type, +Source) is nondet.
%
%   Interface to define a procedure of reading Value and Type of property
%   PropertyDefinition of resource IRI from Source.

:- multifile unmarshal_property/5.

%!  delete_property(+IRI, +PropertyDefinition, +Sink) is det.
%
%   Interface to define a procedure of deleting all values of property
%   PropertyDefinition from resource IRI from Sink.

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

:- rdf_meta create_resource(r, t, t, -).
:- rdf_meta applicable_shapes(t, -).
:- rdf_meta create_resource(r, t, t, t, -).
:- rdf_meta oslc_resource(r, t, -).
:- rdf_meta applicable_shapes(r, -, -).
:- rdf_meta copy_resource(r, r, -, -, -).
:- rdf_meta delete_resource(r, -).

:- rdf_meta rdfType(r).
:- rdf_meta oslcInstanceShape(r).

rdfType(rdf:type).
oslcInstanceShape(oslcs:oslcInstanceShape).

%!  create_resource(+IRI, +Types, +Properties, +Sink) is det.
%
%   Create OSLC resource IRI of given Types with given Properties in Sink.
%   Associate all resource shapes that are applicable to Types with the resource.
%   See create_resource/5 for manual specification of resource shapes.

create_resource(IRI, Types, Properties, Sink) :-
  applicable_shapes(Types, Shapes),
  create_resource(IRI, Types, Shapes, Properties, Sink).

%!  create_resource(+IRI, +Types, +Shapes, +Properties, +Sink) is det.
%
%   Similar to create_resource/4, but the associate only resource shapes
%   from given list of Shapes, which can be empty.

create_resource(IRI, Types, Shapes, Properties, Sink) :-
  must_be(atom, IRI),
  must_be(list(atom), Types),
  must_be(list(atom), Shapes),
  must_be(list(ground), Properties),
  must_be(ground, Sink),
  rdf_transaction((
    delete_property(IRI, _, Sink),
    rdfType(RT),
    marshal_list_property(IRI, RT, Types, _, Sink),
    oslcInstanceShape(OIS),
    write_property(IRI, OIS, Shapes, _, Sink),
    oslc_resource0(IRI, Shapes, Properties, Sink)
  )).

%!  applicable_shapes(+Types, -Shapes) is det.
%
%   True if Shapes is a list of resource shapes applicable to OSLC
%   resources of given list of Types.

applicable_shapes(Types, Shapes) :-
  must_be(list(atom), Types),
  findall(ResourceShape, (
    member(Type, Types),
    rdf(ResourceShape, rdf:type, oslc:'ResourceShape'),
    once((
      \+ rdf(ResourceShape, oslc:describes, _)
    ; rdf(ResourceShape, oslc:describes, Type)
    ))
  ), Shapes).

%!  oslc_resource(+IRI, +Properties, +SourceSink) is det.
%
%   Read and/or write properties of an OSLC resource IRI from/to SourceSink.
%   Check rules from all applicable resource shapes associated with IRI.
%   Properties is a list of terms =|Key=Value|=, where =Key= must
%   correspond to a subject of =oslc:name= in the corresponding property's
%   shape, or be a property IRI. If =Value= is a variable, it will be
%   unified with the value (if exists) of the property =Key=. If =Value=
%   is grounded it will be set as a value of property =Key=. If =Value=
%   is an empty list, property =Key= will be removed from resource IRI.
%   It is allowed to combine read and write in a single call, however,
%   each property may appear in Properties only once.

oslc_resource(IRI, Properties, SourceSink) :-
  applicable_shapes(IRI, Shapes, SourceSink),
  rdf_transaction(
    oslc_resource0(IRI, Shapes, Properties, SourceSink)
  ).

%!  applicable_shapes(+IRI, -Shapes, +Source) is det.
%
%   True if Shapes is a list of resource shapes applicable to OSLC
%   resource IRI from Source.

applicable_shapes(IRI, Shapes, Source) :-
  oslcInstanceShape(OIS),
  read_property(IRI, OIS, ReadShapes, _, Source),
  rdfType(RT),
  unmarshal_list_property(IRI, RT, Types, _, Source),
  findall(ResourceShape, (
    member(ResourceShape, ReadShapes),
    once((
      \+ rdf(ResourceShape, oslc:describes, _)
    ; member(Type, Types),
      rdf(ResourceShape, oslc:describes, Type)
    ))
  ), Shapes).

oslc_resource0(_, _, [], _) :- !.

oslc_resource0(IRI, Shapes, [Property=Value|RemainingProperties], SourceSink) :-
  atom_string(Property, SProperty),
  once((
    member(ResourceShape, Shapes),
    rdf(ResourceShape, oslc:property, PropertyResource),
    once(rdf(PropertyResource, oslc:name, SProperty^^xsd:string)),
    ( var(Value)
    -> read_property(IRI, PropertyResource, _, Value, SourceSink)
    ; once(rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition)),
      delete_property(IRI, PropertyDefinition, SourceSink),
      write_property(IRI, PropertyResource, _, Value, SourceSink)
    )
  ) ; (
    ( var(Value)
    -> unmarshal_some_property(IRI, Property, Value, _, SourceSink)
    ; delete_property(IRI, Property, SourceSink),
      marshal_some_property(IRI, Property, Value, _, SourceSink)
    )
  )),
  oslc_resource0(IRI, Shapes, RemainingProperties, SourceSink).

% ------------ READ PROPERTY

read_property(IRI, PropertyResource, InternalValue, Value, Source) :-
  once(rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition)),
  unmarshal_list_property(IRI, PropertyDefinition, InternalValue, _, Source),
  check_property(IRI, PropertyResource, _, InternalValue, Value).

check_property(IRI, PropertyResource, Type, InternalValue, Value) :-
  check_occurs(IRI, PropertyResource, InternalValue, Value),
  once(rdf(PropertyResource, oslc:valueType, Type)),
  check_value_type(IRI, PropertyResource, InternalValue, Type).

unmarshal_list_property(IRI, PropertyDefinition, Values, Type, Source) :-
  findall(V,
    unmarshal_property(IRI, PropertyDefinition, V, Type, Source)
  , Values).

unmarshal_some_property(IRI, PropertyDefinition, Values, Type, Source) :-
  unmarshal_list_property(IRI, PropertyDefinition, Object, Type, Source),
  once((
    Object == [Values]
  ; Object == Values
  )).

% ------------ WRITE PROPERTY

write_property(IRI, PropertyResource, InternalValue, Value, Sink) :-
  check_property(IRI, PropertyResource, Type, InternalValue, Value),
  once(rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition)),
  marshal_list_property(IRI, PropertyDefinition, InternalValue, Type, Sink).

marshal_list_property(_, _, [], _, _) :- !.

marshal_list_property(IRI, PropertyDefinition, [V|T], Type, Sink) :-
  marshal_property(IRI, PropertyDefinition, V, Type, Sink),
  marshal_list_property(IRI, PropertyDefinition, T, Type, Sink).

marshal_some_property(IRI, PropertyDefinition, Value, Type, Sink) :-
  ( is_list(Value)
  -> marshal_list_property(IRI, PropertyDefinition, Value, Type, Sink)
  ; marshal_property(IRI, PropertyDefinition, Value, Type, Sink)
  ).

%!  copy_resource(+IRIFrom, +IRITo, +Source, +Sink, +Options) is det.
%
%   Copy OSCL resource IRIFrom in Source to IRITo in Sink recursively,
%   i.e. including all of its local resources (blank nodes). Resource
%   IRITo existed in Sink before copying is deleted. A list of Options
%   may contain:
%
%    * inline(InlineSource)
%    If specified, properties of resource IRIFrom described in a shape
%    as having representaton =oslc:Inline= are dereferenced from
%    InlineSource and inlined as blank nodes in resource IRITo.

copy_resource(IRIFrom, IRITo, Source, Sink, Options) :-
  must_be(atom, IRIFrom),
  must_be(atom, IRITo),
  must_be(ground, Source),
  must_be(ground, Sink),
  applicable_shapes(IRIFrom, Shapes, Source),
  rdf_transaction((
    copy_resource0(IRIFrom, IRITo, Shapes, Source, Sink, Options)
  )).

copy_resource0(IRIFrom, IRITo, Shapes, Source, Sink, Options) :-
  delete_resource(IRITo, Sink),
  forall(
    unmarshal_property(IRIFrom, Property, Value, Type, Source)
  , (
    ( rdf_is_bnode(Value)
    -> rdf_create_bnode(Bnode),
       applicable_shapes(Value, BnodeShapes, Source),
       copy_resource0(Value, Bnode, BnodeShapes, Source, Sink, Options),
       marshal_property(IRITo, Property, Bnode, _, Sink)
    ; ( \+ is_literal_type(Type),
        member(inline(InlineSource), Options),
        member(ResourceShape, Shapes),
        rdf(ResourceShape, oslc:property, PropertyResource),
        rdf(PropertyResource, oslc:propertyDefinition, Property),
        rdf(PropertyResource, oslc:representation, oslc:'Inline')
      -> rdf_create_bnode(Bnode),
         applicable_shapes(Value, BnodeShapes, Source),
         copy_resource0(Value, Bnode, BnodeShapes, InlineSource, Sink, Options),
         marshal_property(IRITo, Property, Bnode, _, Sink)
      ; marshal_property(IRITo, Property, Value, Type, Sink)
      )
    )
  )).

%!  delete_resource(+IRI, +Sink) is det.
%
%   Delete OSCL resource IRI from Sink recursively, i.e. including all
%   of its local resources (blank nodes).

delete_resource(IRI, Sink) :-
  must_be(atom, IRI),
  must_be(ground, Sink),
  rdf_transaction((
    delete_resource0(IRI, Sink)
  )).

delete_resource0(IRI, Sink) :-
  forall((
    unmarshal_property(IRI, _, Value, _, Sink),
    rdf_is_bnode(Value)
  ),
    delete_resource(Value, Sink)
  ),
  delete_property(IRI, _, Sink).
