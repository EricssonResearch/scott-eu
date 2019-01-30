package se.ericsson.cf.scott.sandbox.twins.shelf.xtra;

import org.jetbrains.annotations.NotNull;
import se.ericsson.cf.scott.sandbox.twins.shelf.ServiceProviderInfo;
import se.ericsson.cf.scott.sandbox.twins.shelf.ShelfTwinManager;

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
public class AdaptorHelper {
    private static String parameterFQDN(final String s) {
        return ShelfTwinManager.PACKAGE_ROOT + "." + s;
    }

    public static String p(final String s) {
        return ShelfTwinManager.getContext().getInitParameter(parameterFQDN(s));
    }

    @NotNull
    public static ServiceProviderInfo buildSPInfo(final String serviceProviderId, final String name) {
        final ServiceProviderInfo serviceProviderInfo = new ServiceProviderInfo();
        serviceProviderInfo.serviceProviderId = serviceProviderId;
        serviceProviderInfo.name = name;
        return serviceProviderInfo;
    }
}
