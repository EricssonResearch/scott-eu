package se.ericsson.cf.scott.sandbox.twins.shelf.model;

import eu.scott.warehouse.domains.pddl.Action;
import java.util.Collection;
import org.eclipse.lyo.oslc4j.core.model.IResource;

/**
 * Created on 2018-03-04
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public abstract class ActionContainer<A extends Action> implements Container<A> {

    private final A action;

    public ActionContainer(final A action) {this.action = action;}

    @Override
    public A getResource() {
        return action;
    }

    @Override
    public void collectResources(final Collection<IResource> resources) {
        Container.addIfNotContains(resources, action);
    }
}
