:- module(oslc_resource, [
    oslc_resource/3
]).

:- use_module(library(semweb/rdf11)).

:- rdf_meta oslc_resource(r, t, -).

oslc_resource(_, [], _) :- !.
oslc_resource(URI, [Key=Value|T], Graph) :-
  ( rw_property(URI, Key, Value, Graph)
  -> oslc_resource(URI, T, Graph)
  ;  format(atom(Expected), 'mandatory ~w', Key),
     throw(error(type_error(Expected, Value), _))
  ).

rw_property(URI, Key, optional(Value), Graph) :-
  ignore(rw_property(URI, Key, Value, Graph)).

rw_property(URI, Key, xmlliteral(Value), Graph) :-
  ( var(Value)
  -> rdf(URI, Key, ^^(Value, rdf:'XMLLiteral'))
  ;  ( atom(Value)
     -> rdf_assert(URI, Key, Value^^rdf:'XMLLiteral', Graph)
     ;  throw(error(type_error(atom, Value), _))
     )
  ).

rw_property(URI, Key, string(Value), Graph) :-
  ( var(Value)
  -> rdf(URI, Key, ^^(Value, xsd:string))
  ;  ( atom(Value)
     -> rdf_assert(URI, Key, Value^^xsd:string, Graph)
     ;  throw(error(type_error(atom, Value), _))
     )
  ).

rw_property(URI, Key, integer(Value), Graph) :-
  ( var(Value)
  -> rdf(URI, Key, ^^(Value, xsd:integer))
  ;  ( integer(Value)
     -> rdf_assert(URI, Key, Value^^xsd:integer, Graph)
     ;  throw(error(type_error(integer, Value), _))
     )
  ).


rw_property(URI, Key, resource(Value), Graph) :-
  ( var(Value)
  -> rdf(URI, Key, Value)
  ;  ( rdf_is_resource(Value)
     -> rdf_assert(URI, Key, Value, Graph)
     ;  throw(error(type_error(resource, Value), _))
     )
  ).

rw_property(URI, Key, list(Min, Type, Value), Graph) :-
  ( var(Value)
  -> Y =.. [Type, X],
     findall(X, rw_property(URI, Key, Y, Graph), Value),
     length(Value, Length), Length >= Min
  ;  ( is_list(Value), length(Value, Length), Length >= Min
     -> rw_list_property(URI, Key, Type, Value, Graph)
     ;  format(atom(Expected), 'list of length >= ~w', Min),
        throw(error(type_error(Expected, Value), _))
     )
  ).

rw_list_property(_, _, _, [], _) :- !.
rw_list_property(URI, Key, Type, [H|T], Graph) :-
  Value =.. [Type, H],
  rw_property(URI, Key, Value, Graph),
  rw_list_property(URI, Key, Type, T, Graph).
