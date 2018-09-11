package se.ericsson.cf.scott.sandbox.twins.shelf.model;

import java.lang.reflect.InvocationTargetException;
import java.util.Collection;
import java.util.HashSet;
import javax.xml.datatype.DatatypeConfigurationException;
import org.apache.jena.rdf.model.Model;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
import org.eclipse.lyo.oslc4j.core.model.IResource;
import org.eclipse.lyo.oslc4j.core.model.Link;
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper;
import org.eclipse.lyo.oslc4j.provider.jena.LyoJenaModelException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Created on 2018-03-04
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
// TODO Andrew@2018-03-04: consider codegen instances of this
// TODO Andrew@2018-03-04: consider lazy containerisation of child properties
// TODO Andrew@2018-03-04:
public interface Container<R extends IResource> {
    Logger log = LoggerFactory.getLogger(Container.class);

    R getResource();

    // TODO Andrew@2018-03-04: make it a visitor method
    void collectResources(Collection<IResource> resources);

    default Model toModel() throws InvocationTargetException, DatatypeConfigurationException,
            OslcCoreApplicationException, IllegalAccessException {
        final HashSet<IResource> resources = new HashSet<>();
        collectResources(resources);
        final IResource[] objects = resources.toArray(new IResource[0]);
        return JenaModelHelper.createJenaModel(objects);
    }

    static void addIfNotContains(Collection<IResource> collection, IResource r) {
        if (!collection.contains(r) && r != null) {
            log.debug("Adding resource {} to the collection", r.getAbout());
            collection.add(r);
        } else {
            log.trace("Skipping {}", r);
        }
    }

    static <T extends IResource> T tryFollow(final Model m, final Link link, final Class<T> clazz)
            throws LyoJenaModelException {
        final T resource;
        try {
            resource = JenaModelHelper.followLink(m, link, clazz);
        } catch (IllegalArgumentException e) {
            log.warn(
                    "The link {} can't be followed, returning null instead of a {} instance",
                    link,
                    clazz.getSimpleName());
            return null;
        }
        return resource;
    }
}
