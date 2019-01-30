package eu.scott.warehouse.domainx.containers;

import eu.scott.warehouse.domains.blocksworld.Move;
import eu.scott.warehouse.domains.pddl.Plan;
import eu.scott.warehouse.domains.pddl.Step;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import javax.xml.namespace.QName;
import org.apache.jena.rdf.model.Model;
import org.eclipse.lyo.oslc4j.core.model.IExtendedResource;
import org.eclipse.lyo.oslc4j.core.model.IResource;
import org.eclipse.lyo.oslc4j.provider.jena.LyoJenaModelException;

/**
 * Created on 2018-03-04
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public class PlanContainer implements Container<Plan> {

    private final Plan plan;
    private final List<ActionContainer> actions;

    public PlanContainer(final Plan plan, Model m) throws LyoJenaModelException {
        this.plan = plan;
        // FIXME Andrew@2018-03-04: this membership loop originated not from Leo but our SDK!!!
        cleanUpMembership(this.plan);

        final HashSet<Step> steps = plan.getStep();
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
    private void cleanUpMembership(final IExtendedResource r) {
        final Map<QName, Object> extendedProperties = r.getExtendedProperties();
        final QName offendingProperty = new QName("http://www.w3.org/2000/01/rdf-schema#",
                                                  "member",
                                                  "rfds");
        // TODO Andrew@2018-03-04: remove once Bug 531986 is fixed
        extendedProperties.remove(offendingProperty);
//        final ArrayList<Object> members = (ArrayList<Object>) extendedProperties.get(
//                offendingProperty);
//        // remove itself!
//        members.remove(r);
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
