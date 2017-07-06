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
  oslc_resource_shape/5,
  oslc_allowed_values/3,
  oslc_property/18,
  oslc_comment/9,
  oslc_discussion/4
]).

:- use_module(library(semweb/rdf_library)).
:- use_module(library(oslc_resource)).

:- rdf_register_prefix(oslc, 'http://open-services.net/ns/core#').
:- rdf_register_prefix(eos, 'http://ontology.cf.ericsson.net/eos#').

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslc).
:- rdf_load_library(eos).

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
:- rdf_meta oslc_resource_shape(r, -, t, t, -).
:- rdf_meta oslc_allowed_values(r, t, -).
:- rdf_meta oslc_property(r, -, -, r, t, r, -, -, -, -, r, r, t, -, r, t, r, -).
:- rdf_meta oslc_comment(r, -, r, -, -, -, r, r, -).
:- rdf_meta oslc_discussion(r, r, t, -).


oslc_service_provider_catalog(URI, Title, Description, Publisher, Domains,
                              ServiceProviders, ServiceProviderCatalogs,
                              OauthConfigurations, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = property(oslc:'Exactly-one', resource, oslc:'ServiceProviderCatalog'),
      dcterms:title = property(oslc:'Zero-or-one', rdf:'XMLLiteral', Title),
      dcterms:description = property(oslc:'Zero-or-one', rdf:'XMLLiteral', Description),
      dcterms:publisher = property(oslc:'Zero-or-one', resource, Publisher),
      oslc:domain = property(oslc:'Zero-or-many', resource, Domains),
      oslc:serviceProvider = property(oslc:'Zero-or-many', resource, ServiceProviders),
      oslc:serviceProviderCatalog = property(oslc:'Zero-or-many', resource, ServiceProviderCatalogs),
      oslc:oauthConfiguration = property(oslc:'Zero-or-many', resource, OauthConfigurations)
    ], Graph)
  ).

/*
test :-
  rdf_unload_graph(user),
  oslc_service_provider_catalog(spc,call(oslc_api:test2),des,pub,[d1,d2],[sp],[spc1],[oc],user),
  oslc_service_provider_catalog(spc,T,D,_,_,_,_,_,user),
  format(current_output, 'RES = ~w ~w~n', [T,D]).

test2(_URI, _Key, Value, _Graph) :-
  Value = 'haha'.
*/

oslc_service_provider(URI, Title, Description, Publisher,
                      Services, Details, PrefixDefinitions,
                      OauthConfigurations, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = property(oslc:'Exactly-one', resource, oslc:'ServiceProvider'),
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

oslc_resource_shape(URI, Title, Describes, Properties, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'ResourceShape'),
      dcterms:title = optional(xmlliteral(Title)),
      oslc:describes = list(0, resource, Describes),
      oslc:property = list(0, resource, Properties)
    ], Graph)
  ).

oslc_allowed_values(URI, AllowedValues, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'AllowedValues'),
      oslc:allowedValues = list(0, resource, AllowedValues)
    ], Graph)
  ).

oslc_property(URI, Description, Title, AllowedValues, AllowedValueList, DefaultValue,
              Hidden, IsMemberProperty, Name, MaxSize, Occurs, PropertyDefinition,
              Ranges, ReadOnly, Representation, ValueTypes, ValueShape, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'Property'),
      dcterms:description = optional(xmlliteral(Description)),
      dcterms:title = optional(xmlliteral(Title)),
      oslc:allowedValues = optional(resource(AllowedValues)),
      oslc:allowedValue = list(0, resource, AllowedValueList),
      oslc:defaultValue = optional(resource(DefaultValue)),
      oslc:hidden = optional(boolean(Hidden)),
      oslc:isMemberProperty = optional(boolean(IsMemberProperty)),
      oslc:name = string(Name),
      oslc:maxSize = optional(integer(MaxSize)),
      oslc:occurs = resource(Occurs),
      oslc:propertyDefinition = resource(PropertyDefinition),
      oslc:range = list(0, resource, Ranges),
      oslc:readOnly = optional(boolean(ReadOnly)),
      oslc:representation = optional(resource(Representation)),
      oslc:valueType = list(0, resource, ValueTypes),
      oslc:valueShape = optional(resource(ValueShape))
    ], Graph)
  ).

oslc_comment(URI, Identifier, Creator, Title, Created, Description,
             PartOfDiscussion, InReplyTo, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      rdf:type = resource(oslc:'Comment'),
      dcterms:identifier = string(Identifier),
      dcterms:creator = resource(Creator),
      dcterms:title = optional(xmlliteral(Title)),
      dcterms:created = datetime(Created),
      dcterms:description = xmlliteral(Description),
      oslc:partOfDiscussion = resource(PartOfDiscussion),
      oslc:inReplyTo = optional(resource(InReplyTo))
    ], Graph)
  ).

oslc_discussion(URI, DiscussionAbout, Comments, Graph) :-
  nonvar(URI),
  rdf_transaction(
    oslc_resource(URI, [
      oslc:discussionAbout = resource(DiscussionAbout),
      oslc:comment = list(0, resource, Comments)
    ], Graph)
  ).
