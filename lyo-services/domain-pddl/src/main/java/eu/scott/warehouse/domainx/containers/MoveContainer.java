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

import eu.scott.warehouse.domains.blocksworld.Block;
import eu.scott.warehouse.domains.blocksworld.LocationOrBlock;
import eu.scott.warehouse.domains.blocksworld.Move;
import java.util.Collection;
import org.apache.jena.rdf.model.Model;
import org.eclipse.lyo.oslc4j.core.model.IResource;


public class MoveContainer extends ActionContainer<Move> {
    private final Block           block;
    private final LocationOrBlock from;
    private final LocationOrBlock to;

    public MoveContainer(final Move action, final Model m) {
        super(action);

        block = Container.tryFollow(m, action.getMoveB(), Block.class);
        from = Container.tryFollow(m, action.getMoveX(), LocationOrBlock.class);
        to = Container.tryFollow(m, action.getMoveY(), LocationOrBlock.class);
    }

    @Override
    public void collectResources(final Collection<IResource> collection) {
        super.collectResources(collection);

        Container.addIfNotContains(collection, block);
        Container.addIfNotContains(collection, from);
        Container.addIfNotContains(collection, to);
    }
}
