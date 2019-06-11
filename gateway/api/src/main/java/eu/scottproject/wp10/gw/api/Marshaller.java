package eu.scottproject.wp10.gw.api;

import java.util.Collection;
import java.util.Optional;

/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
 */
public interface Marshaller<T> {
    Optional<T> fromBytes(byte[] bytes);
    byte[] toBytes(T obj);
}
