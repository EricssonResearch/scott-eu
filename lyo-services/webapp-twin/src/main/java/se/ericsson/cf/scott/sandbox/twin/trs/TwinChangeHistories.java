package se.ericsson.cf.scott.sandbox.twin.trs;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import javax.servlet.http.HttpServletRequest;
import org.eclipse.lyo.oslc4j.core.model.IResource;
import org.eclipse.lyo.oslc4j.trs.server.ChangeHistories;
import org.eclipse.lyo.oslc4j.trs.server.HistoryData;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
public class TwinChangeHistories extends ChangeHistories {
    // TODO Andrew@2018-09-04: extract a common superclass with the WhcCH
    private final static Logger log = LoggerFactory.getLogger(TwinChangeHistories.class);

    private final List<HistoryData> history = new ArrayList<>();
    private final Map<URI, IResource> trackedResourceMap = new HashMap<>();
    private final MqttClient client;
    private final String topic;

    /**
     *
     * @param client
     * @param topic
     * @param baseUpdateInterval in ms
     */
    public TwinChangeHistories(MqttClient client, String topic, final long baseUpdateInterval) {
        super(baseUpdateInterval);
        this.client = client;
        this.topic = topic;
    }

    @Override
    public HistoryData[] getHistory(final HttpServletRequest httpServletRequest,
        final Date dateAfter) {
        // TODO Andrew@2018-02-26: less expensive implementation
        // TODO Andrew@2018-02-26: consider switching the caller to use lists
        final Date filterDate = dateAfter == null ? new Date(0) : dateAfter;
        return history.stream()
                      .filter(d -> d.getTimestamp().after(filterDate))
                      .toArray(HistoryData[]::new);
    }

    // TODO Andrew@2018-09-04: move to the interface
    public boolean isTracked(URI about) {
        return trackedResourceMap.containsKey(about);
    }

    // TODO Andrew@2018-09-04: move to the abstract class / impl
    public void addResource(IResource resource) {
        final Date now = new Date();
        final String type = isTracked(resource) ? HistoryData.MODIFIED : HistoryData.CREATED;
        final HistoryData historyData = HistoryData.getInstance(now, resource.getAbout(), type);
        history.add(historyData);
        trackedResourceMap.put(resource.getAbout(), resource);
        try {
            super.buildBaseResourcesAndChangeLogs(null);
        } catch (URISyntaxException e) {
            log.error("Something went wrong with the URIs", e);
        }
    }

    private boolean isTracked(final IResource resource) {
        return isTracked(resource.getAbout());
    }

    // TODO Andrew@2018-09-04: move to the abstr class / impl
    public void deleteResource(URI about) {
        throw new UnsupportedOperationException();
    }

    //    @Override
//    protected void newChangeEvent(final ChangeEvent ce) {
//        log.info("New ChangeEvent: {}", ce);
//
//        IResource resource = getResourceForChangeEvent(ce);
//        // FIXME Andrew@2018-03-13: inline
//        MqttMessage message = buildMqttMessage(ce, resource);
//        try {
//            if (!client.isConnected()) {
//                client.connect();
//            }
//            client.publish(topic, message);
//        } catch (MqttException e) {
//            log.error("Can't publish the message to the MQTT channel", e);
//        }
//    }

//    private IResource getResourceForChangeEvent(final ChangeEvent ce) {
//        return trackedResourceMap.get(ce.getChanged());
//    }

//    private MqttMessage buildMqttMessage(ChangeEvent changeEvent, IResource trackedResource) {
//        try {
//            Model changeEventJenaModel;
//            if (trackedResource != null) {
//                log.warn("FAT PING will take place for the {}", changeEvent);
//                changeEventJenaModel = JenaModelHelper.createJenaModel(new Object[]{changeEvent,
//                    trackedResource});
//
//            } else {
//                log.warn("ACHTUNG inlining is impossible, the resource is NULL for {}",
//                         changeEvent);
//                changeEventJenaModel = JenaModelHelper.createJenaModel(new Object[]{changeEvent});
//            }
//            MqttMessage message = new MqttMessage();
//            message.setPayload(marshalJenaModel(changeEventJenaModel));
//            return message;
//        } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException | DatatypeConfigurationException | OslcCoreApplicationException e) {
//            throw new IllegalArgumentException(e);
//        }
//    }
//
//    private byte[] marshalJenaModel(final Model m) {
//        final ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
//        RDFDataMgr.write(byteArrayOutputStream, m, RDFFormat.JSONLD_FLAT);
//        return byteArrayOutputStream.toByteArray();
//    }

}
