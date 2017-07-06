:- module(oslc_resource, [
    oslc_resource/3
]).

:- use_module(library(semweb/rdf_library)).
:- use_module(library(semweb/rdf11)).

:- rdf_register_prefix(pl, 'http://ontology.cf.ericsson.net/prolog#').

:- rdf_meta oslc_resource(r, t, -).

oslc_resource(_, [], _) :- !.
oslc_resource(URI, [Key=Value|T], Graph) :-
  compound(Value),
  ( rw_property(URI, Key, Value, Graph)
  -> oslc_resource(URI, T, Graph)
  ;  format(atom(Expected), 'mandatory ~w', Key),
     throw(error(type_error(Expected, Value), _))
  ).

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

rw_property(URI, Key, xmlliteral(Value), Graph) :-
  ( var(Value)
  -> ( rdf(URI, Key, Value^^rdf:'XMLLiteral', Graph)
     ; r_property(URI, Key, Value, Graph)
     )
  ;  ( atomic(Value)
     -> rdf_assert(URI, Key, Value^^rdf:'XMLLiteral', Graph)
     ;  w_property(URI, Key, Value, Graph)
     ;  throw(error(type_error(atomic, Value), _))
     )
  ).

rw_property(URI, Key, string(Value), Graph) :-
  ( var(Value)
  -> ( rdf(URI, Key, Value^^xsd:string)
     ; r_property(URI, Key, Value, Graph)
     )
  ;  ( atomic(Value)
     -> rdf_assert(URI, Key, Value^^xsd:string, Graph)
     ;  w_property(URI, Key, Value, Graph)
     ;  throw(error(type_error(atomic, Value), _))
     )
  ).

rw_property(URI, Key, integer(Value), Graph) :-
  ( var(Value)
  -> ( rdf(URI, Key, Value^^xsd:integer)
     ; r_property(URI, Key, Value, Graph)
     )
  ;  ( integer(Value)
     -> rdf_assert(URI, Key, Value^^xsd:integer, Graph)
     ;  w_property(URI, Key, Value, Graph)
     ;  throw(error(type_error(integer, Value), _))
     )
  ).

rw_property(URI, Key, boolean(Value), Graph) :-
  ( var(Value)
  -> ( rdf(URI, Key, Value^^xsd:boolean)
     ; r_property(URI, Key, Value, Graph)
     )
  ;  ( member(Value, [true, false])
     -> rdf_assert(URI, Key, Value^^xsd:boolean, Graph)
     ;  w_property(URI, Key, Value, Graph)
     ;  throw(error(type_error(boolean, Value), _))
     )
  ).

rw_property(URI, Key, datetime(Value), Graph) :-
  ( var(Value)
  -> ( rdf(URI, Key, Value^^xsd:dateTime)
     ; r_property(URI, Key, Value, Graph)
     )
  ;  ( atomic(Value), parse_time(Value, iso_8601, _)
     -> rdf_assert(URI, Key, Value^^xsd:dateTime, Graph)
     ;  w_property(URI, Key, Value, Graph)
     ;  throw(error(type_error(datetime, Value), _))
     )
  ).

rw_property(URI, Key, resource(Value), Graph) :-
  ( var(Value)
  -> ( r_property(URI, Key, Value, Graph)
     ; rdf(URI, Key, Value)
     ), rdf_is_resource(Value)
  ;  rdf_is_resource(Value),
     ( w_property(URI, Key, Value, Graph)
     ; rdf_assert(URI, Key, Value, Graph)
     )
  )
  ; throw(error(type_error(resource, Value), _)).

rw_property(URI, Key, list(Min, Type, Value), Graph) :-
  ( var(Value)
  -> ( r_property(URI, Key, Value, Graph)
     ; Y =.. [Type, X],
       findall(X, rw_property(URI, Key, Y, Graph), Value)
     ),
     length(Value, Length),
     Length >= Min
  ;  is_list(Value),
     length(Value, Length),
     Length >= Min,
     ( w_property(URI, Key, Value, Graph)
     ; rw_list_property(URI, Key, Type, Value, Graph)
     )
  )
  ; format(atom(Expected), 'list of length >= ~w', Min),
    throw(error(type_error(Expected, Value), _)).

rw_list_property(_, _, _, [], _) :- !.
rw_list_property(URI, Key, Type, [H|T], Graph) :-
  Value =.. [Type, H],
  rw_property(URI, Key, Value, Graph),
  rw_list_property(URI, Key, Type, T, Graph).
