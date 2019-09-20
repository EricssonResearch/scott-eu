/*
 * Copyright (c) 2018  Ericsson Research and others
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

import java.lang.reflect.InvocationTargetException;
import java.util.Collection;
import java.util.HashSet;
import javax.xml.datatype.DatatypeConfigurationException;
import org.apache.jena.rdf.model.Model;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
import org.eclipse.lyo.oslc4j.core.model.IResource;
import org.eclipse.lyo.oslc4j.core.model.Link;
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


// TODO Andrew@2018-03-04: consider codegen instances of this
// TODO Andrew@2018-03-04: consider lazy containerisation of child properties
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

    static <T extends IResource> T tryFollow(final Model m, final Link link, final Class<T> clazz) {
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
