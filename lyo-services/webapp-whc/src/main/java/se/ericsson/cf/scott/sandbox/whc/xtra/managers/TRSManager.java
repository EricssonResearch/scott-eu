package se.ericsson.cf.scott.sandbox.whc.xtra.managers;

import com.google.common.collect.Lists;
import eu.scott.warehouse.domains.pddl.Plan;
import java.net.URI;
import java.time.Duration;
import java.util.Collection;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Executors;
import org.apache.jena.rdf.model.Model;
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper;
import org.eclipse.lyo.oslc4j.provider.jena.LyoJenaModelException;
import org.eclipse.lyo.oslc4j.trs.client.config.TrsConsumerConfiguration;
import org.eclipse.lyo.oslc4j.trs.client.config.TrsProviderConfiguration;
import org.eclipse.lyo.oslc4j.trs.client.util.TrsBasicAuthOslcClient;
import org.eclipse.lyo.oslc4j.trs.client.util.TrsConsumerUtils;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.whc.xtra.AdaptorHelper;
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerManager;
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerResourcesFactory;
import se.ericsson.cf.scott.sandbox.whc.xtra.planning.PlanRequestBuilder;
import se.ericsson.cf.scott.sandbox.whc.xtra.planning.PlanRequestHelper;
import se.ericsson.cf.scott.sandbox.whc.xtra.planning.PlanRequestHelperJava;
import se.ericsson.cf.scott.sandbox.whc.xtra.trs.WhcChangeHistories;

/**
 * TODO
 *
 * @version $version-stub$
 * @since TODO
 */
public class TRSManager {

    private final static Logger log = LoggerFactory.getLogger(TRSManager.class);
    private final static Map<URI, Object[]> plans = new ConcurrentHashMap<>();

    public static void initTRSClient(final MqttClient mqttClient) {
        final TrsConsumerConfiguration consumerConfig = new TrsConsumerConfiguration(
            AdaptorHelper.p(AdaptorHelper.KB_QUERY_PROP),
            AdaptorHelper.p(AdaptorHelper.KB_UPDATE_PROP), null, null, new TrsBasicAuthOslcClient(),
            AdaptorHelper.getMqttClientId(), Executors.newSingleThreadScheduledExecutor());
        final Collection<TrsProviderConfiguration> providerConfigs = Lists.newArrayList(
            TrsProviderConfiguration.forMQTT(mqttClient,
                AdaptorHelper.p(AdaptorHelper.MQTT_TOPIC_PROP)));
        TrsConsumerUtils.buildHandlersSequential(consumerConfig, providerConfigs);
    }

    public static void initTRSServer(final MqttClient mqttClient) {
        // TODO Andrew@2018-07-18: figure out how the change history works over MQTT
        WarehouseControllerManager.setChangeHistories(
            new WhcChangeHistories(mqttClient, AdaptorHelper.p(AdaptorHelper.MQTT_TOPIC_PROP),
                Duration.ofMinutes(5).toMillis()));
    }

    public static void triggerSamplePlanning() {
        fetchPlanForProblem("sample-problem-request.ttl");
    }

    public static void triggerPlanningForRegisteredTwins() {
        // FIXME Andrew@2019-02-18: implement
        log.warn("New planning is not implemented yet");

        final PlanRequestHelper planRequestHelper = new PlanRequestHelper();
        final PlanRequestBuilder requestBuilder = new PlanRequestBuilder();
        requestBuilder.domainFromModel(planRequestHelper.genDomain())
                      .warehouseSize(100, 100)
                      .robotsActive(10)
                      .robotsInactive(5)
                      .build();
    }

    public static Map<URI, Object[]> getPlans() {
        return plans;
    }

    private static void fetchPlanForProblem(final String resourceName) {
        final Model problemModel = PlanRequestHelperJava.getProblem(resourceName);

        Model planModel = PlanRequestHelperJava.requestPlan(problemModel);
        final Plan plan;
        try {
            plan = extractPlan(planModel);
            log.debug("Received plan: {}", plan);
            final Object[] planResources = PlanRequestHelperJava.getPlanResources(planModel, plan);
            final URI key = plan.getAbout();
            if (plans.containsKey(key)) {
                throw new IllegalStateException("The plan with the same URI has already been fetched");
            }
            getPlans().put(key, planResources);
            // TODO: check the hack from Yash
            // TODO Andrew@2018-02-23: here the plan needs to be put through lyo store update
            WarehouseControllerManager.getChangeHistories().addResource(plan);
        } catch (LyoJenaModelException e) {
            log.error("Cannot unmarshal the Plan");
        }
    }

    @NotNull
    private static Plan extractPlan(final Model planModel) throws LyoJenaModelException {
        final Plan plan = JenaModelHelper.unmarshalSingle(planModel, Plan.class);
        // TODO Andrew@2018-02-26: extract method
        final String planId = String.valueOf(getPlans().size() + 1);
        plan.setAbout(
            WarehouseControllerResourcesFactory.constructURIForPlan(AdaptorHelper.DEFAULT_SP_ID,
                planId));
        return plan;
    }

}
