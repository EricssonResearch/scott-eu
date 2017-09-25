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

:- module(oslc_ontology, [
  register_ontology/1,
  reload_ontologies/2
]).

:- use_module(library(semweb/rdf_library)).

%!  register_ontology(+Ontology) is det.
%
%   Register Ontology (using Ontology as identifier from in Manifest.ttl) to be
%   managed by the OSLC server under URI having form of <BaseURI>/<Ontology>.

register_ontology(Ontology) :-
  assertz(ontology(Ontology)).

%!  reload_ontologies(+OldBaseURI, +NewBaseURI) is det.
%
%   Unloads managed ontologies from OldBaseURI and unregisters corresponding
%   prefixes, and then loads these ontologies and registers new prefixes
%   under NewBaseURI.

reload_ontologies(OldBaseURI, NewBaseURI) :-
  ( current_predicate(ontology/1)
  -> forall(ontology(Ontology), (
       ( nonvar(OldBaseURI)
       -> atom_concat(OldBaseURI, Ontology, OldBaseName),
          rdf_unload_graph(OldBaseName)
       ; true
       ),
       atom_concat(NewBaseURI, Ontology, BaseName),
       atom_concat(BaseName, '#', Source), % the named graph will have name without #
       rdf_load_library(Ontology, [claimed_source(Source)]),
       atom_concat(BaseName, '/', PrefixURI), % newly registered prefix always ends with /
       rdf_register_prefix(Ontology, PrefixURI, [force(true)])
     ))
  ; true
  ).
