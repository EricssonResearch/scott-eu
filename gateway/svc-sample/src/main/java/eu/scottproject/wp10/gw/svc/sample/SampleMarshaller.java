package eu.scottproject.wp10.gw.svc.sample;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import eu.scottproject.wp10.gw.api.Marshaller;
import java.io.IOException;
import java.util.Optional;

/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
 */
public class SampleMarshaller implements Marshaller<WorkerMessageResource> {

    private final ObjectMapper mapper = new ObjectMapper();

    public Optional<WorkerMessageResource> fromBytes(final byte[] bytes) {
        final WorkerMessageResource value;
        try {
            value = mapper.readValue(bytes, WorkerMessageResource.class);
            return Optional.of(value);
        } catch (IOException e) {
            e.printStackTrace();
            return Optional.empty();
        }
    }

    public byte[] toBytes(final WorkerMessageResource resource) {
        try {
            return mapper.writeValueAsBytes(resource);
        } catch (JsonProcessingException e) {
            // TODO Andrew@2018-10-23: can I throw some of those up the chain, like IO or marshalling
            throw new IllegalArgumentException(e);
        }
    }
}
