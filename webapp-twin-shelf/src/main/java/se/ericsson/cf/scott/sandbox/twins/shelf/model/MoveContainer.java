package se.ericsson.cf.scott.sandbox.twins.shelf.model;

import eu.scott.warehouse.domains.blocksworld.Block;
import eu.scott.warehouse.domains.blocksworld.LocationOrBlock;
import eu.scott.warehouse.domains.blocksworld.Move;
import java.util.Collection;
import org.apache.jena.rdf.model.Model;
import org.eclipse.lyo.oslc4j.core.model.IResource;
import org.eclipse.lyo.oslc4j.provider.jena.LyoJenaModelException;

/**
 * Created on 2018-03-04
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public class MoveContainer extends ActionContainer<Move> {
    private final Block           block;
    private final LocationOrBlock from;
    private final LocationOrBlock to;

    public MoveContainer(final Move action, final Model m) throws LyoJenaModelException {
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
