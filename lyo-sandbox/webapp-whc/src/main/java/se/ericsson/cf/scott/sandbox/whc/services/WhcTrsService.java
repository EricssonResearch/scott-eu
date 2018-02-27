package se.ericsson.cf.scott.sandbox.whc.services;

import org.eclipse.lyo.oslc4j.core.OSLC4JUtils;
import org.eclipse.lyo.oslc4j.trs.provider.ChangeHistories;
import org.eclipse.lyo.oslc4j.trs.provider.service.TrackedResourceSetService;
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
        return WhcChangeHistories.INSTANCE;
    }

    @Override
    protected String getServiceBase() {
        return WhcChangeHistories.INSTANCE.getServiceBase();
//        // TODO Andrew@2018-02-26: check if it's the right one
//        final String servletPath = OSLC4JUtils.getServletURI();
//        return servletPath;
    }
}
