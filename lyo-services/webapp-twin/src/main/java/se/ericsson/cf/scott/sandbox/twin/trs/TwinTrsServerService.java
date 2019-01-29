package se.ericsson.cf.scott.sandbox.twin.trs;

import org.eclipse.lyo.oslc4j.core.OSLC4JUtils;
import org.eclipse.lyo.oslc4j.trs.server.ChangeHistories;
import org.eclipse.lyo.oslc4j.trs.server.service.TrackedResourceSetService;
import se.ericsson.cf.scott.sandbox.twin.TwinAdaptorHelper;

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
public class TwinTrsServerService extends TrackedResourceSetService {

    @Override
    protected ChangeHistories getChangeHistories() {
        return TwinAdaptorHelper.getChangeHistories();
    }

    @Override
    protected String getServiceBase() {
        // TODO Andrew@2018-09-04: refactor into the superclass
        return OSLC4JUtils.getServletURI();
    }
}
