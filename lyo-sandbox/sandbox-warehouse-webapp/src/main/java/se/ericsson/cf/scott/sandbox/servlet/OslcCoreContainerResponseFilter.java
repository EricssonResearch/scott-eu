package se.ericsson.cf.scott.sandbox.servlet;

import com.google.common.collect.Lists;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import javax.servlet.Filter;
import javax.servlet.FilterChain;
import javax.servlet.FilterConfig;
import javax.servlet.ServletException;
import javax.servlet.ServletRequest;
import javax.servlet.ServletResponse;
import javax.servlet.annotation.WebFilter;
import javax.servlet.http.HttpServletResponse;
import javax.servlet.http.HttpServletResponseWrapper;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Created on 2017-06-27
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
@WebFilter(servletNames = "JAX-RS Servlet")
public class OslcCoreContainerResponseFilter implements Filter {
    private final static Logger log = LoggerFactory.getLogger(
            OslcCoreContainerResponseFilter.class);

    @Override
    public void init(final FilterConfig filterConfig) throws ServletException {
        log.info("init");
    }

    @Override
    public void doFilter(final ServletRequest request, final ServletResponse response,
            final FilterChain chain) throws IOException, ServletException {
        log.info("doFilter");
        chain.doFilter(request, new OslcCoreHeaderResponseWrapper((HttpServletResponse) response));
    }

    @Override
    public void destroy() {
        log.info("destroy");
    }

    private class OslcCoreHeaderResponseWrapper extends HttpServletResponseWrapper {
        public OslcCoreHeaderResponseWrapper(final HttpServletResponse response) {
            super(response);
            addHeader("OSLC-Core-Version", "2.0");
        }
    }
}
