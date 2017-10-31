/*
Copyright 2017 Ericsson AB

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

:- module('500-oslc_prolog', []).

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
