package se.ericsson.cf.scott.sandbox.whc.managers;

import com.google.common.collect.Lists;
import eu.scott.warehouse.domains.blocksworld.Move;
import eu.scott.warehouse.domains.pddl.Action;
import eu.scott.warehouse.domains.pddl.Plan;
import eu.scott.warehouse.domains.pddl.Step;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URI;
import java.time.Duration;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Executors;
import javax.xml.namespace.QName;
import org.apache.http.HttpResponse;
import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.entity.ByteArrayEntity;
import org.apache.http.impl.client.HttpClientBuilder;
import org.apache.jena.rdf.model.Model;
import org.apache.jena.rdf.model.ModelFactory;
import org.apache.jena.riot.Lang;
import org.apache.jena.riot.RDFDataMgr;
import org.apache.jena.riot.RDFFormat;
import org.eclipse.lyo.oslc4j.core.model.IResource;
import org.eclipse.lyo.oslc4j.trs.client.config.TrsConsumerConfiguration;
import org.eclipse.lyo.oslc4j.trs.client.config.TrsProviderConfiguration;
import org.eclipse.lyo.oslc4j.trs.client.util.TrsBasicAuthOslcClient;
import org.eclipse.lyo.oslc4j.trs.client.util.TrsConsumerUtils;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.whc.AdaptorHelper;
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerManager;
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerResourcesFactory;
import se.ericsson.cf.scott.sandbox.whc.trs.WhcChangeHistories;

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
public class TRSManager {

    private final static Logger log = LoggerFactory.getLogger(TRSManager.class);
    private final static Map<URI, Object[]> plans = new ConcurrentHashMap<>();

    public static void fetchPlanForProblem(final String resourceName) {
        final Model problemModel = getProblem(resourceName);

        Model planModel = requestPlan(problemModel);
        final Plan plan = extractPlan(planModel);
        log.debug("Received plan: {}", plan);
        final Object[] planResources = getPlanResources(planModel, plan);
        final URI key = plan.getAbout();
        if (plans.containsKey(key)) {
            throw new IllegalStateException("The plan with the same URI has already been fetched");
        }
        getPlans().put(key, planResources);
        // TODO: check the hack from Yash
        // TODO Andrew@2018-02-23: here the plan needs to be put through lyo store update

        WarehouseControllerManager.getChangeHistories().addResource(plan);
    }

    public static void initTRSClient(final MqttClient mqttClient) {
        final TrsConsumerConfiguration consumerConfig = new TrsConsumerConfiguration(
            AdaptorHelper.p("kb.query_uri"), AdaptorHelper.p("kb.update_uri"), null, null,
            new TrsBasicAuthOslcClient(), AdaptorHelper.getMqttClientId(),
            Executors.newSingleThreadScheduledExecutor()
        );
        // FIXME Andrew@2019-01-29: MQTT_BROKER_PNAME is not an MQTT topic!!!
        final Collection<TrsProviderConfiguration> providerConfigs = Lists.newArrayList(
                TrsProviderConfiguration.forMQTT(mqttClient, AdaptorHelper.MQTT_BROKER_PNAME));
        TrsConsumerUtils.buildHandlersSequential(consumerConfig, providerConfigs);
    }

    public static void initTRSServer(final MqttClient mqttClient) {
        // TODO Andrew@2018-07-18: figure out how the change history works over MQTT
        WarehouseControllerManager.setChangeHistories(new WhcChangeHistories(mqttClient,
                                                                             AdaptorHelper
                                                                                     .p(AdaptorHelper.MQTT_BROKER_PNAME),
                                                                             Duration.ofMinutes(5)
                                                                                     .toMillis()
        ));
    }

    public static Map<URI, Object[]> getPlans() {
        return plans;
    }

    // TODO Andrew@2018-07-30: move to PlanManager

    private static Model getProblem(final String resourceName) {
        return AdaptorHelper.loadJenaModelFromResource(resourceName, Lang.TURTLE);
    }

    private static Model planForProblem(final Model problemModel) {
        log.trace("Problem request\n{}", AdaptorHelper.jenaModelToString(problemModel));
        try {
            final InputStream response = requestPlanManually(problemModel);
            final Model responsePlan = ModelFactory.createDefaultModel();
            RDFDataMgr.read(responsePlan, response, Lang.TURTLE);
            log.info("Plan response\n{}", AdaptorHelper.jenaModelToString(responsePlan));
            return responsePlan;
        } catch (IOException e) {
            log.error("Something went wrong", e);
            throw new IllegalStateException(e);
        }
    }

    // TODO Andrew@2018-07-30: move to PlanManager
    private static InputStream requestPlanManually(final Model problemModel) throws IOException {

        final ByteArrayOutputStream out = new ByteArrayOutputStream();
        RDFDataMgr.write(out, problemModel, RDFFormat.TURTLE_BLOCKS);

        HttpClient client = HttpClientBuilder.create().build();
        HttpPost post = new HttpPost(AdaptorHelper.p("planner.cf_uri"));

        post.setHeader("Content-type", AdaptorHelper.MIME_TURTLE);
        post.setHeader("Accept", AdaptorHelper.MIME_TURTLE);
        post.setEntity(new ByteArrayEntity(out.toByteArray()));

        HttpResponse response = client.execute(post);
        return response.getEntity().getContent();
    }

    @NotNull
    private static Object[] getPlanResources(final Model planModel, final Plan plan) {
        final ArrayList<IResource> planResources = new ArrayList<>();
        planResources.add(plan);
        // TODO Andrew@2018-02-23: why not getSteps?
        final HashSet<Step> planSteps = plan.getStep();
        for (Step step : planSteps) {
            step.setOrder((Integer) step.getExtendedProperties()
                                        .getOrDefault(new QName(AdaptorHelper.NS_SHACL, "order"),
                                                      null
                                        ));
            final IResource action = AdaptorHelper.navTry(
                    planModel, step.getAction(), Action.class, Move.class);
            planResources.add(step);
            planResources.add(action);
            log.info("Step {}: {}", step.getOrder(), action);
        }
        return planResources.toArray(new Object[0]);
    }

    @NotNull
    private static Plan extractPlan(final Model planModel) {
        final Plan plan = AdaptorHelper.fromJenaModelSingle(planModel, Plan.class);
        // TODO Andrew@2018-02-26: extract method
        final String planId = String.valueOf(getPlans().size() + 1);
        plan.setAbout(
                WarehouseControllerResourcesFactory.constructURIForPlan(AdaptorHelper.DEFAULT_SP_ID,
                                                                        planId
                ));
        return plan;
    }

    @NotNull
    private static Model requestPlan(final Model problemModel) {
        Model planModel = planForProblem(problemModel);
        AdaptorHelper.skolemize(planModel);
        return planModel;
    }
}
