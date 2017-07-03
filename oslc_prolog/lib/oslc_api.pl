:- module(oslc_api, [
    oslc_resource/3,
    oslc_service_provider_catalog/9
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdf_library)).

:- rdf_register_prefix(oslc, 'http://open-services.net/ns/core#').

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslc).

:- rdf_meta oslc_resource(r, t, -).
:- rdf_meta oslc_service_provider_catalog(r, -, -, r, t, t, t, t, -).

oslc_resource(_, [], _) :- !.

oslc_resource(URI, [Key=TV|T], Graph) :-
  TV =.. [Type, Value],
  rw_property(URI, Key, Type, Value, Graph), !,
  oslc_resource(URI, T, Graph).

rw_property(URI, Key, string, Value, Graph) :-
  ( var(Value)
  -> rdf(URI, Key, ^^(Value, _))
  ;  rdf_assert(URI, Key, Value^^xsd:string, Graph)
  ).

rw_property(URI, Key, resource, Value, Graph) :-
  ( var(Value)
  -> rdf(URI, Key, Value)
  ;  ( rdf_is_resource(Value)
     -> rdf_assert(URI, Key, Value, Graph)
     ;  throw(error(type_error(resource, Value), _))
     )
  ).

rw_property(URI, Key, list, TV, Graph) :-
  TV =.. [Type, Value],
  ( var(Value)
  -> findall(X, rw_property(URI, Key, Type, X, Graph), Value)
  ;  ( is_list(Value)
     -> rw_list_property(URI, Key, Type, Value, Graph)
     ;  throw(error(type_error(list, Value), _))
     )
  ).

rw_list_property(_, _, _, [], _) :- !.

rw_list_property(URI, Key, Type, [H|T], Graph) :-
  rw_property(URI, Key, Type, H, Graph),
  rw_list_property(URI, Key, Type, T, Graph).

oslc_service_provider_catalog(URI, Title, Description, Publisher, Domains,
                              ServiceProviders, ServiceProviderCatalogs,
                              OauthConfigurations, Graph) :-
  nonvar(URI),
  oslc_resource(URI, [
    rdf:type = list(resource([owl:'NamedIndividual', oslc:'ServiceProviderCatalog'])),
    dcterms:title = string(Title),
    dcterms:description = string(Description),
    dcterms:publisher = resource(Publisher),
    oslc:domain = list(resource(Domains)),
    oslc:serviceProvider = list(resource(ServiceProviders)),
    oslc:serviceProviderCatalog = list(resource(ServiceProviderCatalogs)),
    oslc:oauthConfiguration = list(resource(OauthConfigurations))
  ], Graph).

% oslc_api:oslc_service_provider_catalog(q,tit,des,pub,[d1,d2],[a,b,c,d],[spc1],[oc1],user)
% oslc_api:oslc_service_provider_catalog(q,Title,Description,Publisher,Domains,SPs,SPCs,OCs,user)
