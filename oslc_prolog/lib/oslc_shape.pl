:- module(oslc_shape, [
  check_property_occurs/3
]).

:- use_module(library(semweb/rdf_library)).
:- use_module(library(oslc_error)).

:- rdf_register_prefix(oslc, 'http://open-services.net/ns/core#').
:- rdf_register_prefix(oslcs, 'http://ontology.cf.ericsson.net/oslc_shapes#').
:- rdf_register_prefix(oslcp, 'http://ontology.cf.ericsson.net/oslc_prolog#').

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslc).
:- rdf_load_library(oslcs).
:- rdf_load_library(oslcp).


check_property_occurs(IRI, PropertyResource, Values) :-
  rdf(PropertyResource, oslc:occurs, Occurs),
  rdf_global_term([oslc:'Zero-or-one',
                   oslc:'Zero-or-many',
                   oslc:'One-or-many',
                   oslc:'Exactly-one'],
                  [ZO, ZM, OM, EO]),
  member(Occurs, [ZO, ZM, OM, EO]),
  length(Values, X),
  ( X == 0
  -> once((
       Occurs == ZO
     ; Occurs == ZM
     ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
       oslc_error("Property ~w of resource ~w must be specified", [PropertyDefinition, IRI])
     ))
  ; ( X > 1
    -> once((
         Occurs == OM
       ; Occurs == ZM
       ; rdf(PropertyResource, oslc:propertyDefinition, PropertyDefinition),
         oslc_error("Property ~w of resource ~w must not occur more than once", [PropertyDefinition, IRI])
       ))
    ; true
    )
  ).
