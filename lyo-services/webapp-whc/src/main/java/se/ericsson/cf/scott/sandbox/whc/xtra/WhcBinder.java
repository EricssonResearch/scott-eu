package se.ericsson.cf.scott.sandbox.whc.xtra;

import org.glassfish.hk2.utilities.binding.AbstractBinder;
import se.ericsson.cf.scott.sandbox.whc.xtra.repository.PlanRepository;
import se.ericsson.cf.scott.sandbox.whc.xtra.repository.PlanRepositoryMem;

/**
 * TODO
 *
 * @since TODO
 */
public class WhcBinder extends AbstractBinder {
    @Override
    protected void configure() {
        bind(PlanRepository.class).to(PlanRepositoryMem.class);
    }
}
