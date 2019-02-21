package se.ericsson.cf.scott.sandbox.whc.xtra.planning;

import eu.scott.warehouse.domains.blocksworld.Move;
import eu.scott.warehouse.domains.pddl.Action;
import eu.scott.warehouse.domains.pddl.Plan;
import eu.scott.warehouse.domains.pddl.Step;
import eu.scott.warehouse.lib.OslcHelpers;
import eu.scott.warehouse.lib.RdfHelpers;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Set;
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
import org.eclipse.lyo.oslc4j.provider.jena.LyoJenaModelException;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.whc.xtra.AdaptorHelper;

/**
 * TODO
 *
 * @since TODO
 */
public class PlanRequestHelperJava {
    private final static Logger log = LoggerFactory.getLogger(PlanRequestHelperJava.class);
    
    public static Model getProblem(final String resourceName) {
        return AdaptorHelper.loadJenaModelFromResource(resourceName, Lang.TURTLE);
    }

    private static Model planForProblem(final Model problemModel) {
        log.trace("Problem request\n{}", RdfHelpers.modelToString(problemModel));
        try {
            final InputStream response = requestPlanManually(problemModel);
            final Model responsePlan = ModelFactory.createDefaultModel();
            RDFDataMgr.read(responsePlan, response, Lang.TURTLE);
            log.info("Plan response\n{}", RdfHelpers.modelToString(responsePlan));
            return responsePlan;
        } catch (IOException e) {
            log.error("Something went wrong", e);
            throw new IllegalStateException(e);
        }
    }

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
    public static Object[] getPlanResources(final Model planModel, final Plan plan)
        throws LyoJenaModelException {
        final ArrayList<IResource> planResources = new ArrayList<>();
        planResources.add(plan);
        // TODO Andrew@2018-02-23: why not getSteps?
        final Set<Step> planSteps = plan.getStep();
        for (Step step : planSteps) {
            step.setOrder((Integer) step.getExtendedProperties()
                                        .getOrDefault(new QName(AdaptorHelper.NS_SHACL, "order"),
                                                      null
                                        ));
            final IResource action = OslcHelpers.navTry(
                    planModel, step.getAction(), Action.class, Move.class);
            planResources.add(step);
            planResources.add(action);
            log.info("Step {}: {}", step.getOrder(), action);
        }
        return planResources.toArray(new Object[0]);
    }

    @NotNull
    public static Model requestPlan(final Model problemModel) {
        Model planModel = planForProblem(problemModel);
        RdfHelpers.skolemize(planModel);
        return planModel;
    }
}
