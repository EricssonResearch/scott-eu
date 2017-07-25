:- module(oslc, [
  create_resource/4,
  applicable_shapes/2,
  create_resource/5,
  oslc_resource/2,
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

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslc).

:- rdf_meta create_resource(r, t, t, -).
:- rdf_meta applicable_shapes(t, -).
:- rdf_meta create_resource(r, t, t, t, -).
:- rdf_meta oslc_resource(r, t).
:- rdf_meta oslc_resource(r, t, -).
:- rdf_meta applicable_shapes(r, -, -).
:- rdf_meta copy_resource(r, r, -, -, -).
:- rdf_meta delete_resource(r, -).

:- rdf_meta rdfType(r).
rdfType(rdf:type).

check_iri(NS:Local, IRI) :- !,
  rdf_global_id(NS:Local, IRI).
check_iri(IRI, IRI) :-
  must_be(atom, IRI).

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
  check_iri(IRI, Id),
  must_be(list(atom), Types),
  must_be(list(atom), Shapes),
  must_be(list(ground), Properties),
  must_be(ground, Sink),
  rdf_transaction((
    delete_property(Id, _, Sink),
    rdfType(RT),
    marshal_list_property(Id, RT, Types, _, Sink),
    oslcInstanceShape(OIS),
    write_property(Id, OIS, Shapes, _, Sink),
    oslc_resource0(Id, Shapes, Properties, Sink),
    check_resource(Id, Shapes, Sink)
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

%!  oslc_resource(+IRI, +Properties) is det.
%
%   Similar to oslc_resource/3, but tries to autodetect named graph
%   =Graph= where resource IRI is defined and use =|rdf(Graph)|=
%   for _SourceSink_.

oslc_resource(IRI, Properties) :-
  check_iri(IRI, Id),
  autodetect_resource_graph(Id, Graph),
  oslc_resource(Id, Properties, rdf(Graph)).

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
  check_iri(IRI, Id),
  oslcInstanceShape(OIS),
  read_property(Id, OIS, ReadShapes, _, Source),
  rdfType(RT),
  unmarshal_list_property(Id, RT, Types, _, Source),
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
  check_iri(IRI, Id),
  atom_string(Property, SProperty),
  once((
    member(ResourceShape, Shapes),
    rdf(ResourceShape, oslc:property, PropertyResource),
    once((
      rdf(PropertyResource, oslc:name, SProperty^^xsd:string)
    ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
      rdf_equal(Property, PropertyDefinition)
    )),
    ( var(Value)
    -> read_property(Id, PropertyResource, _, Value, SourceSink)
    ; once((
        nonvar(PropertyDefinition)
      ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition)
      )),
      delete_property(Id, PropertyDefinition, SourceSink),
      write_property(Id, PropertyResource, _, Value, SourceSink)
    )
  ) ; (
    ( var(Value)
    -> unmarshal_some_property(Id, Property, Value, _, SourceSink)
    ; delete_property(Id, Property, SourceSink),
      marshal_some_property(Id, Property, Value, _, SourceSink)
    )
  )),
  oslc_resource0(Id, Shapes, RemainingProperties, SourceSink).

% ------------ CHECK PROPERTY

check_property(IRI, PropertyResource, Type, InternalValue, Value) :-
  check_occurs(IRI, PropertyResource, InternalValue, Value),
  check_value_type(IRI, PropertyResource, InternalValue, Type).

% ------------ READ PROPERTY

read_property(IRI, PropertyResource, InternalValue, Value, Source) :-
  once(rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition)),
  unmarshal_list_property(IRI, PropertyDefinition, InternalValue, _, Source),
  check_property(IRI, PropertyResource, _, InternalValue, Value).

unmarshal_property(_, _, _, _, []) :- !.

unmarshal_property(IRI, PropertyDefinition, V, Type, [Source|OtherSources]) :-
  unmarshal_property(IRI, PropertyDefinition, V, Type, Source),
  unmarshal_property(IRI, PropertyDefinition, V, Type, OtherSources).

unmarshal_list_property(IRI, PropertyDefinition, Values, Type, Source) :-
  findall(V,
    unmarshal_property(IRI, PropertyDefinition, V, Type, Source)
  , Values).

unmarshal_some_property(IRI, PropertyDefinition, Values, Type, Source) :-
  unmarshal_list_property(IRI, PropertyDefinition, Object, Type, Source),
  once((
    Object = [Values]
  ; Object = Values
  )).

% ------------ WRITE PROPERTY

write_property(IRI, PropertyResource, InternalValue, Value, Sink) :-
  check_property(IRI, PropertyResource, Type, InternalValue, Value),
  once(rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition)),
  marshal_list_property(IRI, PropertyDefinition, InternalValue, Type, Sink).

marshal_property(_, _, _, _, []) :- !.

marshal_property(IRI, PropertyDefinition, V, Type, [Sink|OtherSinks]) :-
  marshal_property(IRI, PropertyDefinition, V, Type, Sink),
  marshal_property(IRI, PropertyDefinition, V, Type, OtherSinks).

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
%
%    * properties(Properties)
%    Copy only properties specified in Properties. The format of the
%    Properties structure is defined by the =properties= term in the
%    following BNF grammar:
%    ==
%      properties   ::= property ("," property)*
%      property     ::= identifier | wildcard | nested_prop
%      nested_prop  ::= (identifier | wildcard) "{" properties "}"
%      wildcard     ::= "*"
%      identifier   ::= prefix ":" name
%      prefix, name ::= /* everything until one of :*,{} */
%    ==
%
%    * prefix(Prefix)
%    Define a set of prefixes to be used in _properties_ option. The
%    syntax of Prefix is defined by the =prefix_defs= term in the
%    following BNF grammar:
%    ==
%      prefix_defs ::= prefix_def ("," prefix_def)*
%      prefix_def  ::= prefix "=" uri_ref_esc
%      prefix      ::= /* everything until = */
%      uri_ref_esc ::= /* an angle bracket-delimited URI reference
%                      in which > and \ are \-escaped. */
%    ==

copy_resource(IRIFrom, IRITo, Source, Sink, Options) :-
  check_iri(IRIFrom, IdFrom),
  check_iri(IRITo, IdTo),
  must_be(ground, Source),
  must_be(ground, Sink),
  applicable_shapes(IdFrom, Shapes, Source),
  ( selectchk(prefix(Prefix), Options, RestOptions1)
  -> parse_prefix(Prefix, PrefixList),
     O1 = [prefix(PrefixList)|RestOptions1]
  ; O1 = Options
  ),
  ( selectchk(properties(Properties), O1, RestOptions2)
  -> parse_properties(Properties, PropertyList, PrefixList),
     O2 = [properties(PropertyList)|RestOptions2]
  ; O2 = O1
  ),
  rdf_transaction((
    copy_resource0(IdFrom, IdTo, Shapes, Source, Sink, O2)
  )).

parse_prefix(Prefix, Structure) :-
  atom_chars(Prefix, CPrefix),
  prefixes(Structure, CPrefix, []).

prefixes([P|Ps]) --> prefix_def(P), ( [','], prefixes(Ps) ; [], { Ps = [] } ).
prefix_def([P,U]) --> prefix(P), ['='], uri(U).
prefix(P) --> prefix_letters(Pl), { atom_chars(P, Pl), ! }.
prefix_letters([L|Ls]) --> prefix_letter(L), prefix_letters(Ls).
prefix_letters([]) --> [].
prefix_letter(L) --> [L], { L \== '=' }.

uri(U) --> ['<'], uri_letters(Ul), { atom_chars(U, Ul), ! }, ['>'].
uri_letters([L|Ls]) --> uri_letter(L), uri_letters(Ls).
uri_letters([]) --> [].
uri_letter('>') --> ['\\'], ['>'].
uri_letter('\\') --> ['\\'], ['\\'].
uri_letter(L) --> [L], { L \== '>' }.

parse_properties(Properties, Structure, Prefixes) :-
  atom_chars(Properties, CProperties),
  properties(Structure, Prefixes, CProperties, []).

properties([P|Ps], Prefixes) --> property(P, Prefixes), ( [','], properties(Ps, Prefixes) ; [], { Ps = [] } ).
property(P, Prefixes) --> identifier(P, Prefixes) ; wildcard(P) ; nested_prop(P, Prefixes).
nested_prop(NP, Prefixes) --> ( identifier(N, Prefixes) ; wildcard(N) ), ['{'], properties(Ps, Prefixes), ['}'],
                    { NP =.. [N,Ps] }.
wildcard('*') --> ['*'].
identifier(I, Prefixes) --> word(Prefix), [':'], word(Name),
                            { once((
                                nonvar(Prefixes),
                                member([Prefix,Uri], Prefixes)
                              ; rdf_current_prefix(Prefix, Uri)
                              )),
                              atom_concat(Uri, Name, I)
                            }.
word(W) --> word_letters(Wl), { atom_chars(W, Wl), ! }.
word_letters([L|Ls]) --> letter(L), word_letters(Ls).
word_letters([]) --> [].
letter(L) --> [L], { \+ member(L, [':','*',',','{','}']) }.

copy_resource0(IRIFrom, IRITo, Shapes, Source, Sink, Options) :-
  delete_resource(IRITo, Sink),
  % TODO: probably we should combine copy with checking and make copy the default checking procedure for all purposes
  check_resource(IRIFrom, Shapes, Source),
  ( selectchk(properties(PropertyList), Options, RestOptions)
  -> true
  ; RestOptions = Options
  ),
  forall(
    unmarshal_property(IRIFrom, Property, Value, Type, Source)
  , (
    ( rdf_is_bnode(Value)
    -> copy_bnode(Value, Source, Source, Sink, Options, IRITo, Property, PropertyList, RestOptions)
    ; ( \+ is_literal_type(Type),
        member(inline(InlineSource), Options),
        member(ResourceShape, Shapes),
        rdf(ResourceShape, oslc:property, PropertyResource),
        rdf(PropertyResource, oslc:propertyDefinition, Property),
        rdf(PropertyResource, oslc:representation, oslc:'Inline')
      -> copy_bnode(Value, Source, InlineSource, Sink, Options, IRITo, Property, PropertyList, RestOptions)
      ; copy_property(Value, Sink, IRITo, Property, Type, PropertyList)
      )
    )
  )).

check_resource(IRI, Shapes, Source) :-
  forall((
    member(ResourceShape, Shapes),
    rdf(ResourceShape, oslc:property, PropertyResource),
    rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition)
  ), (
    unmarshal_list_property(IRI, PropertyDefinition, Values, Type, Source),
    check_property(IRI, PropertyResource, Type, Values, _),
    ( rdf(PropertyResource, oslc:representation, oslc:'Inline')
    -> forall(
         member(Value, Values),
         (
           applicable_shapes(Value, BnodeShapes, Source),
           check_resource(Value, BnodeShapes, Source)
         )
       )
    ; true
    )
  )).

copy_bnode(Value, ShapeSource, Source, Sink, Options, IRITo, Property, PropertyList, RestOptions) :-
  ( ( var(PropertyList),
      NewOptions = Options
    ; once((
        member('*'(SubList), PropertyList)
      ; P =.. [Property, SubList],
        member(P, PropertyList)
      )),
      NewOptions = [properties(SubList)|RestOptions]
    )
  -> rdf_create_bnode(Bnode),
     applicable_shapes(Value, BnodeShapes, ShapeSource),
     copy_resource0(Value, Bnode, BnodeShapes, Source, Sink, NewOptions),
     marshal_property(IRITo, Property, Bnode, _, Sink)
  ; true
  ).

copy_property(Value, Sink, IRITo, Property, Type, PropertyList) :-
  ( ( var(PropertyList)
    ; once((
        member('*', PropertyList)
      ; member(Property, PropertyList)
      ))
    )
  -> marshal_property(IRITo, Property, Value, Type, Sink)
  ; true
  ).

%!  delete_resource(+IRI, +Sink) is det.
%
%   Delete OSCL resource IRI from Sink recursively, i.e. including all
%   of its local resources (blank nodes).

delete_resource(IRI, Sink) :-
  check_iri(IRI, Id),
  must_be(ground, Sink),
  rdf_transaction((
    ignore(delete_resource0(Id, Sink))
  )).

delete_resource0(IRI, Sink) :-
  forall((
    unmarshal_property(IRI, _, Value, _, Sink),
    rdf_is_bnode(Value)
  ),
    delete_resource0(Value, Sink)
  ),
  delete_property(IRI, _, Sink).
