:- module(oslc_rdf, []).

:- use_module(library(semweb/rdf11)).

:- rdf_meta literal_types(t).

% ------------ RDF SOURCE / SINK

oslc:marshal_property(_, _, [], _, rdf(_)) :- !.

oslc:marshal_property(IRI, PropertyDefinition, [V|T], Type, rdf(Graph)) :-
  atom(Graph),
  must_be(atom, IRI),
  must_be(atom, PropertyDefinition),
  literal_types(LiteralTypes),
  ( nonvar(Type), member(Type, LiteralTypes)
  -> rdf_assert(IRI, PropertyDefinition, V^^Type, Graph)
  ; rdf_assert(IRI, PropertyDefinition, V, Graph)
  ),
  oslc:marshal_property(IRI, PropertyDefinition, T, Type, rdf(Graph)).

literal_types([ rdf:'XMLLiteral',
                xsd:string,
                xsd:integer,
                xsd:float,
                xsd:double,
                xsd:decimal,
                xsd:dateTime,
                xsd:boolean
              ]).

oslc:unmarshal_property(IRI, PropertyDefinition, Value, Type, rdf) :-
  must_be(atom, IRI),
  must_be(atom, PropertyDefinition),
  findall(V, (
    rdf(IRI, PropertyDefinition, X),
    once((
      rdf_is_literal(X),
      X = V^^Type
    ; V = X
    ))
  ), Value).

oslc:unmarshal_property(IRI, PropertyDefinition, Value, Type, rdf(Graph)) :-
  atom(Graph),
  must_be(atom, IRI),
  must_be(atom, PropertyDefinition),
  findall(V, (
    rdf(IRI, PropertyDefinition, X, Graph),
    once((
      rdf_is_literal(X),
      X = V^^Type
    ; V = X
    ))
  ), Value).

oslc:delete_property(IRI, PropertyDefinition, rdf(Graph)) :-
  atom(Graph),
  must_be(atom, IRI),
  rdf_retractall(IRI, PropertyDefinition, _, Graph).
