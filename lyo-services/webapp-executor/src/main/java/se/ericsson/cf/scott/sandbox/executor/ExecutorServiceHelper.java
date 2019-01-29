package se.ericsson.cf.scott.sandbox.executor;

import java.util.Collections;
import java.util.List;
import java.util.Map;
import javax.servlet.ServletContext;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * TODO
 *
 * @since TODO
 */
public class ExecutorServiceHelper {
    private final static Logger log = LoggerFactory.getLogger(ExecutorServiceHelper.class);

    public static String p(final String s) {
        final String fqdn = pFQDN(s);
        final String parameter = PlanExecutorManager.getServletContext().getInitParameter(fqdn);
        log.trace("{}={" + "}", fqdn, parameter);
        return parameter;
    }

    static String pFQDN(final String key) {
        final String base = PlanExecutorManager.class.getPackage().getName();
        log.trace("Using '{}' as a parameter base", base);
        return base.trim() + "." + key.trim();
    }

    static void dumpEnvVars() {
        final Map<String, String> environmentVariables = System.getenv();
        final StringBuilder builder = new StringBuilder();
        environmentVariables.forEach((k, v) -> builder.append(String.format("%s='%s';\n", k, v)));
        log.debug("The environment variables of the adaptor:\n{}", builder);
    }

    // TODO Andrew@2018-08-03: move to the common helper class
    static void dumpInitParams(ServletContext c) {
        log.trace("Context initialisation parameters:");
        final List<String> parameterNames = Collections.list(c.getInitParameterNames());
        for (String p : parameterNames) {
            log.trace("\t{}='{}'", p, c.getInitParameter(p));
        }
        log.trace("---");
    }
}
