package eu.scott.warehouse.lib;

import eu.scott.warehouse.domains.containers.ActionContainer;
import eu.scott.warehouse.domains.containers.PlanContainer;
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
import org.eclipse.lyo.oslc4j.core.model.IResource;
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper;
import org.eclipse.lyo.oslc4j.provider.jena.LyoJenaModelException;
import org.eclipse.lyo.oslc4j.trs.client.exceptions.JenaModelException;
import org.eclipse.lyo.oslc4j.trs.client.handlers.ChangeEventListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Created on 2018-02-27
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
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

/*
    // FIXME Andrew@2018-03-04: hangs infinitely with a Plan
    private static String lyoResourceToString(final IResource... resources)
            throws JenaModelException {
        try {
            return jenaModelToString(JenaModelHelper.createJenaModel(resources));
        } catch (DatatypeConfigurationException | IllegalAccessException | OslcCoreApplicationException | InvocationTargetException e) {
            // FIXME Andrew@2018-02-27: move this exception to Lyo
            throw new JenaModelException(e);
        }
    }
*/

    @Override
    public void handleChangeEvent(final ChangeEvent changeEvent, final Model trsResourceModel) {
        if (changeEvent instanceof Deletion) {
            // FIXME Andrew@2018-02-27: this should not involve reflection, but just be an enum +
            // getType()
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


            /*
             * Okay, now we are redefining the architecture and we might want to go away from the
             * LWM2M for the purposes of the demo.
             */
            // FIXME Andrew@2018-03-06: reevaluate the plan below depending on whether we use lwm2m

            // TODO Andrew@2018-03-05: define a single-threaded executor or init a thread by hand

            // TODO Andrew@2018-03-05: terminate any previously running runnable!
            // TODO Andrew@2018-03-05: add create a PlanExecutionRunnable instance

            // TODO Andrew@2018-03-05: Drain the write requests queue and log their cancellation
            // TODO Andrew@2018-03-05: process all new values before executing plan
            // TODO Andrew@2018-03-05: convert a step of a plan into a WriteRequest(s)
            // TODO Andrew@2018-03-05: block until the step has executed (via SynchronousQueue)
            // or something more advanced than that (ie to handle plan execution error cases)

            // TODO Andrew@2018-03-05: define the same process for handling the new values queue

            /*
             * So we are going to make a dummy plan executor for now while the LWM2M issue are
             * getting ironed out. Once we get an LWM2M comm or any other comm to the ROS and
             * make sure the comm is nicely working with VREP, we can remove this dummy code.
             */
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
