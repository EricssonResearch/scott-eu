package eu.scott.warehouse.lib

import eu.scott.warehouse.domains.scott.ActionExecutionReport
import org.junit.Test

import org.junit.Assert.*

/*
 * Copyright (c) 2019  Ericsson Research and others
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
class LyoModelsTest {

    @Test
    fun hasResource() {
        val graph = """
            @prefix trs:   <http://open-services.net/ns/core/trs#> .
            @prefix rdf:   <http://www.w3.org/1999/02/22-rdf-syntax-ns#> .
            @prefix xsd:   <http://www.w3.org/2001/XMLSchema#> .
            @prefix dcterms: <http://purl.org/dc/terms/> .
            @prefix ldp:   <http://www.w3.org/ns/ldp#> .
            @prefix rdfs:  <http://www.w3.org/2000/01/rdf-schema#> .
            @prefix oslc:  <http://open-services.net/ns/core#> .
            
            <urn:skolem:4dcda8681b4cc13aa992259d0ef0bb7b>
                    a       <http://ontology.cf.ericsson.net/ns/scott-warehouse/move-to-wp> ;
                    <http://ontology.cf.ericsson.net/ns/scott-warehouse/move-to-wp_from>
                            <http://ontology.cf.ericsson.net/ns/scott-warehouse/x4y1> ;
                    <http://ontology.cf.ericsson.net/ns/scott-warehouse/move-to-wp_rb>
                            <http://ontology.cf.ericsson.net/ns/scott-warehouse/rb-2> ;
                    <http://ontology.cf.ericsson.net/ns/scott-warehouse/move-to-wp_to>
                            <http://ontology.cf.ericsson.net/ns/scott-warehouse/x1y3> .
            
            <http://twins.svc:8080/services/twins/robot/rb-2/service2/actionExecutionReports/0834a50a-bf95-4d8b-bbe7-5856ea2a89f1>
                    a       <http://ontology.cf.ericsson.net/ns/scott-warehouse/ActionExecutionReport> ;
                    <http://ontology.cf.ericsson.net/ns/scott-warehouse/action>
                            <urn:skolem:4dcda8681b4cc13aa992259d0ef0bb7b> ;
                    <http://ontology.cf.ericsson.net/ns/scott-warehouse/executionBegin>
                            "2019-07-22T13:00:08.454Z"^^xsd:dateTime ;
                    <http://ontology.cf.ericsson.net/ns/scott-warehouse/executionEnd>
                            "2019-07-22T13:00:08.454Z"^^xsd:dateTime ;
                    <http://ontology.cf.ericsson.net/ns/scott-warehouse/executionSuccess>
                            true .
        """

        val model = RdfHelpers.modelFromIndentedString(graph)

        val hasResource = LyoModels.hasResource(model, ActionExecutionReport::class.java)

        assertTrue("ActionExecutionReport is not found in the RDF graph", hasResource)
    }
}
