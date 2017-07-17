:- module(oslc_rdf, []).

:- use_module(library(semweb/rdf11)).

:- rdf_meta literal_types(t).

% ------------ RDF SOURCE / SINK

oslc:marshal_property(IRI, PropertyDefinition, Value, Type, rdf(Graph)) :-
  literal_types(LiteralTypes),
  ( member(Type, LiteralTypes)
  -> rdf_assert(IRI, PropertyDefinition, Value^^Type, Graph)
  ; rdf_assert(IRI, PropertyDefinition, Value, Graph)
  ).

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
  findall(V, (
    rdf(IRI, PropertyDefinition, X),
    once((
      rdf_is_literal(X),
      X = V^^Type
    ; V = X
    ))
  ), Value).

oslc:unmarshal_property(IRI, PropertyDefinition, Value, Type, rdf(Graph)) :-
  findall(V, (
    rdf(IRI, PropertyDefinition, X, Graph),
    once((
      rdf_is_literal(X),
      X = V^^Type
    ; V = X
    ))
  ), Value).

oslc:remove_property(IRI, PropertyDefinition, rdf) :-
  rdf_retractall(IRI, PropertyDefinition, _).

oslc:remove_property(IRI, PropertyDefinition, rdf(Graph)) :-
  rdf_retractall(IRI, PropertyDefinition, _, Graph).
