package se.ericsson.cf.scott.sandbox.whc.xtra.services;

import org.eclipse.lyo.oslc4j.core.OSLC4JUtils;
import org.eclipse.lyo.oslc4j.trs.server.ChangeHistories;
import org.eclipse.lyo.oslc4j.trs.server.service.TrackedResourceSetService;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerManager;

/**
 * Created on 2018-02-26
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public class WhcTrsService extends TrackedResourceSetService {

    private final static Logger log = LoggerFactory.getLogger(WhcTrsService.class);

    @Override
    protected ChangeHistories getChangeHistories() {
        return WarehouseControllerManager.getChangeHistories();
    }

    @Override
    protected String getServiceBase() {
        final String servletPath = OSLC4JUtils.getServletURI();
        return servletPath;
    }
}
