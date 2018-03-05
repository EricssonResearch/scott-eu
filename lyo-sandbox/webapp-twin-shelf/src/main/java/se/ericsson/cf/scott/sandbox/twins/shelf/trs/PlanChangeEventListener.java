package se.ericsson.cf.scott.sandbox.twins.shelf.trs;

import eu.scott.warehouse.domains.pddl.Plan;
import java.io.StringWriter;
import java.lang.reflect.InvocationTargetException;
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
//
////            log.debug("Received a new or updated Plan:\n{}", lyoResourceToString(plan));
//            log.debug("Received a new or updated Plan: {}", plan);
//            final HashSet<Step> steps = plan.getStep();
//            final List<Step> stepList = steps.stream()
//                                            .sorted(Comparator.comparingInt(Step::getOrder))
//                                            .collect(Collectors.toList());
//            for (Step step : stepList) {
//                // FIXME Andrew@2018-03-04: properly unmarshal any action
//                final Move action = JenaModelHelper.followLink(
//                        trsResourceModel,
//                        step.getAction(),
//                        Move.class
//                );

            try {
                final Model model = planContainer.toModel();
                log.debug(
                        "The Plan contains the following resources in its model:\n{}",
                        jenaModelToString(model));
            } catch (InvocationTargetException | DatatypeConfigurationException |
                    OslcCoreApplicationException | IllegalAccessException e) {
                e.printStackTrace();
            }
//            log.info("Found step {}: {}", step.getOrder(), action);
        } catch (LyoJenaModelException e) {
            log.error("A Plan cannot be built from the TRS change event resource model", e);
        }
    }
}
