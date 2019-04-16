package se.ericsson.cf.scott.sandbox.whc.xtra;

import java.net.URI;
import java.util.UUID;
import org.jetbrains.annotations.NotNull;

public class WhcConfig {
    public static final String KB_QUERY_PROP = "kb.query_uri";
    // Start of user code class_attributes
    // TODO Andrew@2018-07-30: extract into AdaptorConfig (non-static)
    public static final String MQTT_TOPIC_PROP = "trs.mqtt.broker";
    public static final String DEFAULT_SP_ID = "default";
    public static final String NS_SHACL = "http://www.w3.org/ns/shacl#";
    public static final String MIME_TURTLE = "text/turtle";
    public static final String MQTT_CLIENT_ID = "trs-consumer-whc";
    public static final String KB_UPDATE_PROP = "kb.update_uri";
    final static UUID whcUUID = UUID.randomUUID();
    private static URI baseUri = URI.create("http://ontology.cf.ericsson.net/ns/scott-warehouse/");

    @NotNull
    public static URI getBaseUri() {
        return baseUri;
    }
}
