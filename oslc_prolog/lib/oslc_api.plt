:- begin_tests(oslc_api, [
  setup(rdf_unload_graph(oslc_api_test))
  %,cleanup(rdf_unload_graph(oslc_api_test))
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(oslc_api)).

:- rdf_meta assertion(r,t).

:- rdf_register_prefix(test, 'http://ontology.cf.ericsson.net/test#').

assertion(Actual, Expected) :-
  assertion(Actual == Expected).

test(oslc_service_provider_catalog) :-
  oslc_service_provider_catalog(test:s1, "service provider catalog", o3, test:o4, [test:o5, test:o6], [test:o7, test:o8], [test:o9], [test:o10], oslc_api_test),
  assertion(rdf(test:s1, rdf:type, oslc:'ServiceProviderCatalog')),
  oslc_service_provider_catalog(test:s1, A, B, C, D, E, F, G, oslc_api_test),
  assertion(A == "service provider catalog"),
  assertion(B == "o3"),
  assertion(C, test:o4),
  assertion(D, [test:o5, test:o6]),
  assertion(E, [test:o7, test:o8]),
  assertion(F, [test:o9]),
  assertion(G, [test:o10]).


test(oslc_service_provider) :-
  oslc_service_provider(test:s2, "service provider", o3, test:o4, [test:o5, test:o6], [test:o7, test:o8], [test:o9], [test:o10], oslc_api_test),
  assertion(rdf(test:s2, rdf:type, oslc:'ServiceProvider')),
  oslc_service_provider(test:s2, A, B, C, D, E, F, G, oslc_api_test),
  assertion(A == "service provider"),
  assertion(B == "o3"),
  assertion(C, test:o4),
  assertion(D, [test:o5, test:o6]),
  assertion(E, [test:o7, test:o8]),
  assertion(F, [test:o9]),
  assertion(G, [test:o10]).

  /*
  oslc_service_provider_catalog(spc,'service provider catalog',des,pub,[d1,d2],[sp],[spc1],[oc],user),
  oslc_service_provider(sp,'sevice provider',des,pub,[s],[d1,d2,d3],[pd1],[oc2],user),
  oslc_service(s,dom,[cf,cf2],[qc,qc2],[sd,sd2],[cd1,cd2],[u1,u2],user),
  oslc_creation_factory(cf,'creation factory',lab,cr,[rs,rs2],[rt1,rt2],[u3,u4],user),
  oslc_query_capability(qc,'query capability',lab,qb,[rs3,rs4],[rt3,rt4],[u5,u6],user),
  oslc_dialog(sd,'dialog',lab,d,'100px','200px',[rt5],[u7,u8],user),
  oslc_publisher(pub,'publisher',lab,id,icon,user),
  oslc_oauth_configuration(oc, artu, au, oatu, user),
  oslc_response_info(ri,'response info',des,np,5,user),
  oslc_error(er,'404','not found',exterr,user),
  oslc_extended_error(exterr,mi,rel,'50px','60px',user),
  oslc_resource_shape(rs,'resource shape',[de1,de2],[pr,pr2],user),
  oslc_allowed_values(av,[av1,av2],user),
  oslc_property(pr,des,'property',av,[av3],dv,true,false,n,10,o,pd,[r1],false,rep,[vt1,vt2],vs,user),
  oslc_comment(com,id,cr,'comment','2006-12-08T17:29:44+02:00',des,pod,irt,user),
  oslc_discussion(dis,da,[com,com2],user).
  */

:- end_tests(oslc_api).
