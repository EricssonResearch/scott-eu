package se.ericsson.cf.scott.sandbox.clients;

import java.io.IOException;

/**
 * Created on 2017-07-10
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public class OslcClientException extends IOException {
    public OslcClientException() {
    }

    public OslcClientException(final String message) {
        super(message);
    }

    public OslcClientException(final String message, final Throwable cause) {
        super(message, cause);
    }

    public OslcClientException(final Throwable cause) {
        super(cause);
    }
}
