/*
 * Copyright (c) 2018 Ericsson Research and others
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

package eu.scott.warehouse.domainx.containers;

import eu.scott.warehouse.domains.blocksworld.Move;
import eu.scott.warehouse.domains.pddl.Plan;
import eu.scott.warehouse.domains.pddl.Step;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import javax.xml.namespace.QName;
import org.apache.jena.rdf.model.Model;
import org.eclipse.lyo.oslc4j.core.model.IExtendedResource;
import org.eclipse.lyo.oslc4j.core.model.IResource;

public class PlanContainer implements Container<Plan> {

    private final Plan plan;
    private final List<ActionContainer> actions;

    public PlanContainer(final Plan plan, Model m) {
        this.plan = plan;
        // FIXME Andrew@2018-03-04: this membership loop originated not from Leo but our SDK!!!
        cleanUpMembership(this.plan);

        final Set<Step> steps = plan.getStep();
        final List<Step> stepList = steps.stream()
                                         .sorted(Comparator.comparingInt(Step::getOrder))
                                         .collect(Collectors.toList());
        actions = new ArrayList<>();
        for (Step step : stepList) {
            // FIXME Andrew@2018-03-04: properly unmarshal any action
            final Move action = Container.tryFollow(m, step.getAction(), Move.class);
            if (action != null) {
                MoveContainer moveContainer = new MoveContainer(action, m);
                actions.add(moveContainer);
            }
        }
    }

    /**
     * A resource r with the property {@code <r> rdfs:member <r> .} will enter a loop in Lyo JMH
     * if we try to create a model from it.
     * @param r
     */
    // TODO Andrew@2018-03-04: remove once Bug 531986 is fixed
    private void cleanUpMembership(final IExtendedResource r) {
        final Map<QName, Object> extendedProperties = r.getExtendedProperties();
        final QName offendingProperty = new QName("http://www.w3.org/2000/01/rdf-schema#",
                                                  "member",
                                                  "rfds");
        extendedProperties.remove(offendingProperty);
    }

    @Override
    public Plan getResource() {
        return plan;
    }

    public List<ActionContainer> getActions() {
        return actions;
    }

    @Override
    public void collectResources(final Collection<IResource> resources) {
        Container.addIfNotContains(resources, plan);

        for (ActionContainer action : actions) {
            action.collectResources(resources);
        }

    }
}
