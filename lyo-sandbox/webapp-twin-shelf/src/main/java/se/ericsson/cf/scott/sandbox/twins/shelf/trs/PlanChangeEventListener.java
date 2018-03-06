package se.ericsson.cf.scott.sandbox.twins.shelf.trs;

import eu.scott.warehouse.domains.pddl.Plan;
import java.io.StringWriter;
import java.lang.reflect.InvocationTargetException;
import java.time.Duration;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
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
import org.eclipse.lyo.trs.consumer.exceptions.JenaModelException;
import org.eclipse.lyo.trs.consumer.handlers.ChangeEventListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.twins.shelf.ShelfTwinManager;
import se.ericsson.cf.scott.sandbox.twins.shelf.model.ActionContainer;
import se.ericsson.cf.scott.sandbox.twins.shelf.model.PlanContainer;

/**
 * Created on 2018-02-27
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public class PlanChangeEventListener implements ChangeEventListener {
    private final static Logger log = LoggerFactory.getLogger(PlanChangeEventListener.class);

//     TODO Andrew@2018-02-07: submit to the JenaModelHelper
//    private static <T> T[] fromJenaModelTyped(final Model model, Class<T> clazz) {
//        try {
//            final Object[] objects = JenaModelHelper.fromJenaModel(model, clazz);
//            //noinspection unchecked
//            final T[] clazzObjects = (T[]) objects;
//            return clazzObjects;
//        } catch (DatatypeConfigurationException | IllegalAccessException |
//                InvocationTargetException | InstantiationException | OslcCoreApplicationException
//                | NoSuchMethodException | URISyntaxException e) {
//            throw new IllegalArgumentException(e);
//        }
//    }

//    // TODO Andrew@2018-02-07: submit to the JenaModelHelper
//    private static <T> T fromJenaModelSingle(final Model model, Class<T> clazz)
//            throws LyoJenaModelException {
//        final T[] ts = JenaModelHelper.unmarshal(model, clazz);
//        if (ts.length != 1) {
//            throw new IllegalArgumentException("Model shall contain exactly 1 instance of the
// class");
//        }
//        return ts[0];
//    }

    // TODO Andrew@2018-02-27: move it to JenaModelHelper or something of a sort
    private static String jenaModelToString(final Model responsePlan) {
        final StringWriter stringWriter = new StringWriter();
        RDFDataMgr.write(stringWriter, responsePlan, RDFFormat.TURTLE_PRETTY);
        return stringWriter.toString();
    }

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
            final Future<PlanExecutionResult> planExecutionResultFuture = ShelfTwinManager
                    .planExecutorSvc
                    .submit(new Callable<PlanExecutionResult>() {
                        @Override
                        public PlanExecutionResult call() throws Exception {
                            Thread.sleep(5);
                            for (ActionContainer actionContainer : planContainer.getActions()) {
                                log.info("Executing {}", actionContainer.getResource());
                            }
                            return new PlanExecutionResult(true, Duration.ofSeconds(5));
                        }
                    });
            try {
                final PlanExecutionResult planExecutionResult = planExecutionResultFuture.get();
                log.info(
                        "The plan finished correctly within {}s",
                        planExecutionResult.getPlanExecutionTime().getSeconds());
            } catch (InterruptedException | ExecutionException e) {
                log.error("Error executing the plan", e);
            }

            try {
                final Model model = planContainer.toModel();
                log.debug(
                        "The Plan contains the following resources in its model:\n{}",
                        jenaModelToString(model));
            } catch (InvocationTargetException | DatatypeConfigurationException |
                    OslcCoreApplicationException | IllegalAccessException e) {
                e.printStackTrace();
            }
        } catch (LyoJenaModelException e) {
            log.error("A Plan cannot be built from the TRS change event resource model", e);
        }
    }
}
