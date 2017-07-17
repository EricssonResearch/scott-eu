:- module(oslc, [
  register_resource/1,
  oslc_resource/5
]).

:- multifile marshal_property/5.
:- multifile unmarshal_property/5.
:- multifile remove_property/3.

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
:- rdf_meta oslc_resource(r, r, t, -, -).
:- rdf_meta oslcs_rdf_type(r).

:- initialization init.

init :-
  forall(
    rdf(X, rdf:type, oslcp:'PrologResource'),
    register_resource(X)
  ).

%!  register_resource(:PrologResource) is det.
%
%   Creates and exports a predicate to operate PrologResource. The argument must
%   point to a resource of type oslcp:PrologResource. The created predicate will
%   have signature =|module:pred_name(IRI, Options, Source, Sink)|=. Where =module=
%   is Prolog module (prologModule property of PrologResource), =pred_name= is the
%   predicate name of arity 4 (prologPredicate property of PrologResource), IRI
%   is the =IRI= of a resource managed by the API, =Options= is a list of properties
%   having a form of =|Key(Value)|= or =|Key=Value|=, =Source= and =Sink= are the
%   pointers to demarshaller and marshaller respectively (rdf(Graph), where =Graph=
%   is a name of named graph, are standard triple store implementations).

register_resource(PrologResource) :-
  once((
    rdf(PrologResource, oslcp:prologModule, ModuleName^^xsd:string),
    rdf(PrologResource, oslcp:prologPredicate, PredicateName^^xsd:string),
    rdf(PrologResource, oslc:resourceShape, ResourceShape)
  )),
  atom_string(AModuleName, ModuleName),
  atom_string(APredicateName, PredicateName),
  P =.. [APredicateName, Spec, Options, Source, Sink],
  retractall(AModuleName:P),
  assertz((
    AModuleName:P :-
      rdf_global_id(Spec, IRI),
      oslc_resource(IRI, ResourceShape, Options, Source, Sink)
  )),
  AModuleName:export(APredicateName/4).

%!  oslc_resource(:IRI, :ResourceShape, ?Options, ?Source, ?Sink) is det.
%
%   Reads and/or writes properties of an OSLC resource IRI, from/to Source/Sink.
%   The shape of the OSLC resource must be provided in ResourceShape. The Options
%   is a list of properties of the resource in form of =|Key(Value)|= or =|Key=Value|=.
%   The =Key= must correspond to the =oslc:name= property of the property's shape.
%   If =Value= is a variable, it will be unified with the value of the property =Key=
%   unmashalled from the Source. If =Value= is grounded it will be set as a value of
%   =Key= and marshalled to the Sink. If =Value= is an empty list, the value of
%   property =Key= will be removed from the Sink. It is allowed to combine marshalling
%   and unmarshalling of properties in a single call, however, every property may
%   appear only once in Options.

oslc_resource(IRI, ResourceShape, Options, Source, Sink) :-
  rdf_transaction(
    oslc_resource0(IRI, ResourceShape, Options, Source, Sink)
  ).

oslcs_rdf_type(oslcs:rdfType).

oslc_resource0(IRI, ResourceShape, Options, Source, Sink) :-
  oslcs_rdf_type(TypePropertyResource),
  once((
    read_property(IRI, TypePropertyResource, A, Source),
    nonvar(A)
  ; findall(V, rdf(ResourceShape, oslc:describes, V), Resources),
    write_property(IRI, TypePropertyResource, Resources, Sink)
  )),
  findall(P, rdf(ResourceShape, oslc:property, P), Properties),
  oslc_properties(IRI, ResourceShape, Properties, Options, Source, Sink).

% ------------ TRAVERSE PROPERTIES

oslc_properties(_, _, [], _, _, _).

oslc_properties(IRI, ResourceShape, [PropertyResource|T], Options, Source, Sink) :-
  once(rdf(PropertyResource, oslc:name, Property^^xsd:string)),
  atom_string(AProperty, Property),
  ( once((
      H =.. [AProperty, Value], selectchk(H, Options, RemainingOptions)
    ; H = (AProperty = Value), selectchk(H, Options, RemainingOptions)
    ))
  -> ( var(Value)
     -> read_property(IRI, PropertyResource, Value, Source)
     ; write_property(IRI, PropertyResource, Value, Sink)
     ),
     oslc_properties(IRI, ResourceShape, T, RemainingOptions, Source, Sink)
  ; read_property(IRI, PropertyResource, _, Source),
    oslc_properties(IRI, ResourceShape, T, Options, Source, Sink)
  ).

% ------------ READ PROPERTY

read_property(IRI, PropertyResource, Value, Source) :-
  once(rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition)),
  unmarshal_property(IRI, PropertyDefinition, InternalValue, _, Source),
  read_value_list(IRI, PropertyResource, InternalValue),
  check_occurs(IRI, PropertyResource, InternalValue, Value).

read_value_list(_, _, []).

read_value_list(IRI, PropertyResource, [H|T]) :-
  check_value_type(IRI, PropertyResource, H, _),
  read_value_list(IRI, PropertyResource, T).

% ------------ WRITE PROPERTY

write_property(IRI, PropertyResource, Value, Sink) :-
  check_occurs(IRI, PropertyResource, InternalValue, Value),
  once(rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition)),
  remove_property(IRI, PropertyDefinition, Sink),
  write_list(IRI, PropertyResource, PropertyDefinition, InternalValue, Sink).

write_list(_, _, _, [], _).

write_list(IRI, PropertyResource, PropertyDefinition, [H|T], Sink) :-
  check_value_type(IRI, PropertyResource, H, Type),
  marshal_property(IRI, PropertyDefinition, H, Type, Sink),
  write_list(IRI, PropertyResource, PropertyDefinition, T, Sink).
