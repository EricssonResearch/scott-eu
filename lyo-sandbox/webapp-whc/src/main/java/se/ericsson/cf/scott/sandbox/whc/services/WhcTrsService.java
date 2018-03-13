package se.ericsson.cf.scott.sandbox.whc.services;

import org.eclipse.lyo.oslc4j.trs.server.ChangeHistories;
import org.eclipse.lyo.oslc4j.trs.server.service.TrackedResourceSetService;
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerManager;
import se.ericsson.cf.scott.sandbox.whc.trs.WhcChangeHistories;

/**
 * Created on 2018-02-26
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public class WhcTrsService extends TrackedResourceSetService {
    @Override
    protected ChangeHistories getChangeHistories() {
        return WarehouseControllerManager.getChangeHistories();
    }

    @Override
    protected String getServiceBase() {
        return getChangeHistories().getServiceBase();
//        // TODO Andrew@2018-02-26: check if it's the right one
//        final String servletPath = OSLC4JUtils.getServletURI();
//        return servletPath;
    }
}
