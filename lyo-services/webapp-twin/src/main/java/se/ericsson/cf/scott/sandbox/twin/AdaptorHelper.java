package se.ericsson.cf.scott.sandbox.twin;

/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
 */
public class AdaptorHelper {
    // Start of user code class_methods
    private static String parameterFQDN(final String s) {
        return TwinManager.PACKAGE_ROOT + "." + s;
    }

    public static String p(final String s) {
        return TwinManager.getServletContext().getInitParameter(parameterFQDN(s));
    }
}
