:- module(conf_oslc_prolog, []).

/** <module> OSLC Prolog
*/

%:- debug(http(send_request)).
%:- debug(http(open)).
%:- debug(rdf(load)).

:- use_module(library(semweb/rdf_library)).

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslc).
:- rdf_load_library(oslc_shapes).

:- use_module(library(oslc)).
:- use_module(library(oslc_core)).

:- use_module(oslc_prolog(applications/oslc_prolog_server)).
:- use_module(oslc_prolog(applications/oslc_cliopatria_ui)).
