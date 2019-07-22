package eu.scott.warehouse.lib

import eu.scott.warehouse.domains.pddl.Action
import eu.scott.warehouse.domains.pddl.Plan
import eu.scott.warehouse.domains.pddl.Step
import eu.scott.warehouse.domains.scott.DropBelt
import eu.scott.warehouse.domains.scott.ExecutableAction
import eu.scott.warehouse.domains.scott.IExecutableAction
import eu.scott.warehouse.domains.scott.MoveToWp
import eu.scott.warehouse.domains.scott.PickShelf
import org.eclipse.lyo.oslc4j.core.model.IExtendedResource
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.junit.Test

import org.junit.Assert.*
import se.ericsson.cf.scott.sandbox.twin.xtra.PlanExecutionService
import se.ericsson.cf.scott.sandbox.twin.xtra.plans.ExecutionInfo
import java.util.Date

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
class OslcHelpersTest {

    val plan = RdfHelpers.modelFromIndentedString("""
        @prefix dcterms: <http://purl.org/dc/terms/> .
        @prefix rdfs:  <http://www.w3.org/2000/01/rdf-schema#> .
        @prefix j.1:   <http://www.w3.org/ns/shacl#> .
        @prefix j.0:   <http://ontology.cf.ericsson.net/pddl/> .
        @prefix j.2:   <http://ontology.cf.ericsson.net/ns/scott-warehouse/> .
        @prefix oslc:  <http://open-services.net/ns/core#> .
        
        <urn:skolem:74cf0c8bfd98a4487a4dcb948b04b27f>
                a                   j.0:Step ;
                j.0:action          <urn:skolem:7c8ff6d24e839b44794d79ef41449f5b> ;
                j.0:order           "2"^^<http://www.w3.org/2001/XMLSchema#int> ;
                oslc:instanceShape  j.0:StepShape ;
                j.1:order           "2"^^<http://www.w3.org/2001/XMLSchema#int> .
        
        <urn:skolem:09ec255d87df613d84ca969c3cbee3ce>
                a                   j.0:Step ;
                j.0:action          <urn:skolem:215328ca328042f5eb67cbe25555b8c9> ;
                j.0:order           "3"^^<http://www.w3.org/2001/XMLSchema#int> ;
                oslc:instanceShape  j.0:StepShape ;
                j.1:order           "3"^^<http://www.w3.org/2001/XMLSchema#int> .
        
        <urn:skolem:897ef5e03475da9c6ca528f0d7cadf41>
                a                   j.0:Step ;
                j.0:action          <urn:skolem:d027253e75a1310688810942bc6040e0> ;
                j.0:order           "1"^^<http://www.w3.org/2001/XMLSchema#int> ;
                oslc:instanceShape  j.0:StepShape ;
                j.1:order           "1"^^<http://www.w3.org/2001/XMLSchema#int> .
        
        <http://whc.svc:8080/services/serviceProviders/default/service1/plans/1>
                a                   j.0:Plan ;
                j.0:cost            "4.0"^^<http://www.w3.org/2001/XMLSchema#float> ;
                j.0:step            <urn:skolem:897ef5e03475da9c6ca528f0d7cadf41> , <urn:skolem:09ec255d87df613d84ca969c3cbee3ce> , <urn:skolem:80d043e7cdc0b9e31de7da1e36e03d4e> , <urn:skolem:74cf0c8bfd98a4487a4dcb948b04b27f> ;
                j.0:time            "0.0"^^<http://www.w3.org/2001/XMLSchema#float> ;
                oslc:instanceShape  j.0:PlanShape .
        
        <urn:skolem:215328ca328042f5eb67cbe25555b8c9>
                a                    j.2:move-to-wp ;
                j.2:move-to-wp_from  j.2:x1y3 ;
                j.2:move-to-wp_rb    j.2:rb-4 ;
                j.2:move-to-wp_to    j.2:x3y3 .
        
        <urn:skolem:80d043e7cdc0b9e31de7da1e36e03d4e>
                a                   j.0:Step ;
                j.0:action          <urn:skolem:c3ca61f2cb637b11f8b008f9a8869551> ;
                j.0:order           "4"^^<http://www.w3.org/2001/XMLSchema#int> ;
                oslc:instanceShape  j.0:StepShape ;
                j.1:order           "4"^^<http://www.w3.org/2001/XMLSchema#int> .
        
        <urn:skolem:c3ca61f2cb637b11f8b008f9a8869551>
                a                 j.2:drop-belt ;
                j.2:drop-belt-b   j.2:b1 ;
                j.2:drop-belt-cb  j.2:cb1 ;
                j.2:drop-belt-rb  j.2:rb-4 ;
                j.2:drop-belt-wp  j.2:x3y3 .
        
        <urn:skolem:d027253e75a1310688810942bc6040e0>
                a                    j.2:move-to-wp ;
                j.2:move-to-wp_from  j.2:x1y1 ;
                j.2:move-to-wp_rb    j.2:rb-4 ;
                j.2:move-to-wp_to    j.2:x1y3 .
        
        <urn:skolem:7c8ff6d24e839b44794d79ef41449f5b>
                a                  j.2:pick-shelf ;
                j.2:pick-shelf-b   j.2:b1 ;
                j.2:pick-shelf-rb  j.2:rb-4 ;
                j.2:pick-shelf-sh  j.2:sh1 ;
                j.2:pick-shelf-wp  j.2:x1y3 .
    """)

    @Test
    fun navTry() {
        val planResource = JenaModelHelper.unmarshalSingle(plan, Plan::class.java)
        val actionSteps = planResource.step.sortedBy { it.order }
        actionSteps.forEach { step: Step ->
//            val action: IExtendedResource = OslcHelpers.navTry(plan, step.action,
//                ExecutableAction::class.java, DropBelt::class.java, MoveToWp::class.java,
//                PickShelf::class.java)
            val action: IExtendedResource = OslcHelpers.navTry(plan, step.action,
                ExecutableAction::class.java)
        }
    }
}
