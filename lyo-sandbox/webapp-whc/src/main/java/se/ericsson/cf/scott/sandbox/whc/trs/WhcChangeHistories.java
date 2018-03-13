package se.ericsson.cf.scott.sandbox.whc.trs;

import java.io.ByteArrayOutputStream;
import java.lang.reflect.InvocationTargetException;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import javax.servlet.http.HttpServletRequest;
import javax.xml.datatype.DatatypeConfigurationException;
import org.apache.jena.rdf.model.Model;
import org.apache.jena.riot.RDFDataMgr;
import org.apache.jena.riot.RDFFormat;
import org.eclipse.lyo.core.trs.ChangeEvent;
import org.eclipse.lyo.oslc4j.core.OSLC4JUtils;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
import org.eclipse.lyo.oslc4j.core.model.AbstractResource;
import org.eclipse.lyo.oslc4j.core.model.IResource;
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper;
import org.eclipse.lyo.oslc4j.trs.server.ChangeHistories;
import org.eclipse.lyo.oslc4j.trs.server.HistoryData;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Created on 2018-02-26
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public class WhcChangeHistories extends ChangeHistories {

    private final static Logger log = LoggerFactory.getLogger(WhcChangeHistories.class);

    // TODO Andrew@2018-02-26: extract a default "manual" implementation
//    public static final WhcChangeHistories INSTANCE = new WhcChangeHistories();
    private final List<HistoryData> history = new ArrayList<>();
    private final MqttClient client;
    private final String     topic;

    public WhcChangeHistories(MqttClient client, String topic) {
        this.client = client;
        this.topic = topic;
        // FIXME Andrew@2018-02-27: the WhcTrsService shall pass the base on all requests
        this.setServiceBase(OSLC4JUtils.getServletURI());
    }

    @Override
    public HistoryData[] getHistory(final HttpServletRequest httpServletRequest, final Date dateAfter) {
        // TODO Andrew@2018-02-26: less expensive implementation
        // TODO Andrew@2018-02-26: consider switching the caller to use lists
        return history.toArray(new HistoryData[0]);
    }

    @Override
    protected void newChangeEvent(final ChangeEvent ce) {
        log.info("New ChangeEvent: {}", ce);

        // FIXME Andrew@2018-03-13: inline
        MqttMessage message = buildMqttMessage(ce, null);
        try {
            client.publish(topic, message);
        } catch (MqttException e) {
            log.error("Can't publish the message to the MQTT channel", e);
        }
    }

    public void addResource(IResource resource) {
        final Date now = new Date();
        final HistoryData historyData = HistoryData.getInstance(now, resource.getAbout(), HistoryData.CREATED);
        history.add(historyData);
        try {
            super.buildBaseResourcesAndChangeLogs(null);
        } catch (URISyntaxException e) {
            log.error("Something went wrong with the URIs", e);
        }

    }

    private MqttMessage buildMqttMessage(ChangeEvent changeEvent,
            AbstractResource trackedResource) {
        try {
            Model changeEventJenaModel = JenaModelHelper.createJenaModel(new Object[]{changeEvent});
            MqttMessage message = new MqttMessage();
            message.setPayload(marshalJenaModel(changeEventJenaModel));
            return message;
        } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException |
                DatatypeConfigurationException | OslcCoreApplicationException e) {
            throw new IllegalArgumentException(e);
        }
    }

    private byte[] marshalJenaModel(final Model m) {
        final ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
        RDFDataMgr.write(byteArrayOutputStream, m, RDFFormat.JSONLD_COMPACT_FLAT);
        return byteArrayOutputStream.toByteArray();
    }

}
