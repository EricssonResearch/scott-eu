package eu.scott.warehouse.lib;

import eu.scott.warehouse.domainx.containers.ActionContainer;
import eu.scott.warehouse.domainx.containers.PlanContainer;
import eu.scott.warehouse.domains.pddl.Plan;
import java.io.StringWriter;
import java.lang.reflect.InvocationTargetException;
import java.time.Duration;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;
import javax.xml.datatype.DatatypeConfigurationException;
import org.apache.jena.rdf.model.Model;
import org.apache.jena.riot.RDFDataMgr;
import org.apache.jena.riot.RDFFormat;
import org.eclipse.lyo.core.trs.ChangeEvent;
import org.eclipse.lyo.core.trs.Deletion;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper;
import org.eclipse.lyo.oslc4j.provider.jena.LyoJenaModelException;
import org.eclipse.lyo.oslc4j.trs.client.handlers.ChangeEventListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Plans no longer arrive over MQTT, hence this code will not be triggered in the current architecture.
 */
@Deprecated
public class PlanChangeEventListener implements ChangeEventListener {
    private final static Logger log = LoggerFactory.getLogger(PlanChangeEventListener.class);
    private ExecutorService executorService;

    public PlanChangeEventListener(
            final ExecutorService executorService) {
        this.executorService = executorService;
    }

    // TODO Andrew@2018-02-27: move it to JenaModelHelper or something of a sort
    private static String jenaModelToString(final Model responsePlan) {
        final StringWriter stringWriter = new StringWriter();
        RDFDataMgr.write(stringWriter, responsePlan, RDFFormat.TURTLE_PRETTY);
        return stringWriter.toString();
    }

    @Override
    public void handleChangeEvent(final ChangeEvent changeEvent, final Model trsResourceModel) {
        if (changeEvent instanceof Deletion) {
            // TODO Andrew@2018-02-27: this should not involve reflection, but just be an enum +
            // getType() (after inheritance works in JMH)
            log.info("Ignoring deletion event");
            return;
        }
        try {
            final Plan plan = JenaModelHelper.unmarshalSingle(trsResourceModel, Plan.class);
            final PlanContainer planContainer = new PlanContainer(plan, trsResourceModel);

            try {
                final Model model = planContainer.toModel();
                log.debug(
                        "The Plan contains the following resources in its model:\n{}",
                        jenaModelToString(model));
            } catch (InvocationTargetException | DatatypeConfigurationException |
                    OslcCoreApplicationException | IllegalAccessException e) {
                e.printStackTrace();
            }

            final Future<PlanExecutionResult> planExecutionResultFuture = executorService
                    .submit(() -> {
                        Thread.sleep(5);
                        for (ActionContainer actionContainer : planContainer.getActions()) {
                            log.info("Executing {}", actionContainer.getResource());
                        }
                        return new PlanExecutionResult(true, Duration.ofSeconds(5));
                    });
            try {
                final PlanExecutionResult planExecutionResult = planExecutionResultFuture.get();
                log.info(
                        "The plan finished correctly within {}s",
                        planExecutionResult.getPlanExecutionTime().getSeconds());
                // TODO Andrew@2018-03-06: report back the plan has executed successfully
            } catch (InterruptedException | ExecutionException e) {
                log.error("Error executing the plan", e);
            }
        } catch (LyoJenaModelException e) {
            log.error("A Plan cannot be built from the TRS change event resource model", e);
        }
    }
}
