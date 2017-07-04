:- module(oslc_api, [
  oslc_service_provider_catalog/9,
  oslc_service_provider/9,
  oslc_service/8,
  oslc_creation_factory/8,
  oslc_query_capability/8,
  oslc_dialog/9,
  oslc_publisher/6,
  oslc_prefix_definition/4,
  oslc_oauth_configuration/5,
  oslc_response_info/6,
  oslc_error/5,
  oslc_extended_error/6,
  oslc_results/4
]).

:- use_module(library(semweb/rdf_library)).
:- use_module(library(oslc_resource)).

:- rdf_register_prefix(oslc, 'http://open-services.net/ns/core#').

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslc).

:- rdf_meta oslc_service_provider_catalog(r, -, -, r, t, t, t, t, -).
:- rdf_meta oslc_service_provider(r, -, -, r, t, t, t, t, -).
:- rdf_meta oslc_service(r, r, t, t, t, t, r, -).
:- rdf_meta oslc_creation_factory(r, -, -, r, t, t, t, -).
:- rdf_meta oslc_query_capability(r, -, -, r, t, t, t, -).
:- rdf_meta oslc_dialog(r, -, -, r, -, -, t, t, -).
:- rdf_meta oslc_publisher(r, -, -, -, r, -).
:- rdf_meta oslc_prefix_definition(r, -, r, -).
:- rdf_meta oslc_oauth_configuration(r, r, r, r, -).
:- rdf_meta oslc_response_info(r, -, -, r, -, -).
:- rdf_meta oslc_error(r, -, -, r, -).
:- rdf_meta oslc_extended_error(r, r, -, -, -, -).
:- rdf_meta oslc_results(r, r, -, -).

oslc_service_provider_catalog(URI, Title, Description, Publisher, Domains,
                              ServiceProviders, ServiceProviderCatalogs,
                              OauthConfigurations, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'ServiceProviderCatalog'),
      dcterms:title = optional(xmlliteral(Title)),
      dcterms:description = optional(xmlliteral(Description)),
      dcterms:publisher = optional(resource(Publisher)),
      oslc:domain = list(0, resource, Domains),
      oslc:serviceProvider = list(0, resource, ServiceProviders),
      oslc:serviceProviderCatalog = list(0, resource, ServiceProviderCatalogs),
      oslc:oauthConfiguration = list(0, resource, OauthConfigurations)
    ], Graph)
  ).

oslc_service_provider(URI, Title, Description, Publisher,
                      Services, Details, PrefixDefinitions,
                      OauthConfigurations, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'ServiceProvider'),
      dcterms:title = optional(xmlliteral(Title)),
      dcterms:description = optional(xmlliteral(Description)),
      dcterms:publisher = optional(resource(Publisher)),
      oslc:service = list(1, resource, Services),
      oslc:details = list(0, resource, Details),
      oslc:prefixDefinition = list(0, resource, PrefixDefinitions),
      oslc:oauthConfiguration = list(0, resource, OauthConfigurations)
    ], Graph)
  ).

oslc_service(URI, Domain, CreationFactories, QueryCapabilities,
             SelectionDialogs, CreationDialogs, Usages, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'Service'),
      oslc:domain = resource(Domain),
      oslc:creationFactory = list(0, resource, CreationFactories),
      oslc:queryCapability = list(0, resource, QueryCapabilities),
      oslc:selectionDialog = list(0, resource, SelectionDialogs),
      oslc:creationDialog = list(0, resource, CreationDialogs),
      oslc:usage = list(0, resource, Usages)
    ], Graph)
  ).

oslc_creation_factory(URI, Title, Label, Creation, ResourceShapes,
                      ResourceTypes, Usages, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'CreationFactory'),
      dcterms:title = xmlliteral(Title),
      oslc:label = optional(string(Label)),
      oslc:creation = resource(Creation),
      oslc:resourceShape = list(0, resource, ResourceShapes),
      oslc:resourceType = list(0, resource, ResourceTypes),
      oslc:usage = list(0, resource, Usages)
    ], Graph)
  ).

oslc_query_capability(URI, Title, Label, QueryBase, ResourceShapes,
                      ResourceTypes, Usages, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'QueryCapability'),
      dcterms:title = xmlliteral(Title),
      oslc:label = optional(string(Label)),
      oslc:queryBase = resource(QueryBase),
      oslc:resourceShape = list(0, resource, ResourceShapes),
      oslc:resourceType = list(0, resource, ResourceTypes),
      oslc:usage = list(0, resource, Usages)
    ], Graph)
  ).

oslc_dialog(URI, Title, Label, Dialog, HintWidth, HintHeight,
            ResourceTypes, Usages, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'Dialog'),
      dcterms:title = xmlliteral(Title),
      oslc:label = optional(string(Label)),
      oslc:dialog = resource(Dialog),
      oslc:hintWidth = optional(string(HintWidth)),
      oslc:hintHeight = optional(string(HintHeight)),
      oslc:resourceType = list(0, resource, ResourceTypes),
      oslc:usage = list(0, resource, Usages)
    ], Graph)
  ).

oslc_publisher(URI, Title, Label, Identifier, Icon, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'Publisher'),
      dcterms:title = xmlliteral(Title),
      oslc:label = optional(string(Label)),
      dcterms:identifier = string(Identifier),
      oslc:icon = optional(resource(Icon))
    ], Graph)
  ).

oslc_prefix_definition(URI, Prefix, PrefixBase, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'PrefixDefinition'),
      dcterms:title = string(Prefix),
      oslc:label = resource(PrefixBase)
    ], Graph)
  ).

oslc_oauth_configuration(URI, OauthRequestTokenURI, AuthorizationURI,
                         OauthAccessTokenURI, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'OAuthConfiguration'),
      oslc:oauthRequestTokenURI = resource(OauthRequestTokenURI),
      oslc:authorizationURI = resource(AuthorizationURI),
      oslc:oauthAccessTokenURI = resource(OauthAccessTokenURI)
    ], Graph)
  ).

oslc_response_info(URI, Title, Description, NextPage, TotalCount, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'ResponseInfo'),
      dcterms:title = optional(xmlliteral(Title)),
      dcterms:description = optional(xmlliteral(Description)),
      oslc:nextPage = optional(resource(NextPage)),
      oslc:totalCount = optional(integer(TotalCount))
    ], Graph)
  ).

oslc_error(URI, StatusCode, Message, ExtendedError, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'Error'),
      oslc:statusCode = string(StatusCode),
      oslc:message = string(Message),
      oslc:extendedError = optional(resource(ExtendedError))
    ], Graph)
  ).

oslc_extended_error(URI, MoreInfo, Rel, HintWidth, HintHeight, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'ExtendedError'),
      oslc:moreInfo = optional(resource(MoreInfo)),
      oslc:rel = optional(string(Rel)),
      oslc:hintWidth = optional(string(HintWidth)),
      oslc:hintHeight = optional(string(HintHeight))
    ], Graph)
  ).

oslc_results(URI, Resource, Label, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'results'),
      rdf:resource = optional(resource(Resource)),
      oslc:label = optional(string(Label))
    ], Graph)
  ).

% -------------------

oslc_example :-
  rdf_unload_graph(user),
  oslc_service_provider_catalog(spc,'service provider catalog',des,pub,[d1,d2],[sp],[spc1],[oc],user),
  oslc_service_provider(sp,'sevice provider',des,pub,[s],[d1,d2,d3],[pd1],[oc2],user),
  oslc_service(s,dom,[cf,cf2],[qc,qc2],[sd,sd2],[cd1,cd2],[u1,u2],user),
  oslc_creation_factory(cf,'creation factory',lab,cr,[rs1,rs2],[rt1,rt2],[u3,u4],user),
  oslc_query_capability(qc,'query capability',lab,qb,[rs3,rs4],[rt3,rt4],[u5,u6],user),
  oslc_dialog(sd,'dialog',lab,d,'100px','200px',[rt5],[u7,u8],user),
  oslc_publisher(pub,'publisher',lab,id,icon,user),
  oslc_oauth_configuration(oc, artu, au, oatu, user),
  oslc_response_info(ri,'response info',des,np,5,user),
  oslc_error(er,'404','not found',exterr,user),
  oslc_extended_error(exterr,mi,rel,'50px','60px',user),
  oslc_results(results,res,lab,user).
