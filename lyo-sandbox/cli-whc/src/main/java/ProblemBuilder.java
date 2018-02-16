import eu.scott.warehouse.domains.pddl.Domain;
import eu.scott.warehouse.domains.pddl.Function;
import eu.scott.warehouse.domains.pddl.PddlObject;
import eu.scott.warehouse.domains.pddl.PrimitiveType;
import eu.scott.warehouse.domains.pddl.Problem;
import java.io.StringWriter;
import java.lang.reflect.InvocationTargetException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Collection;
import java.util.HashSet;
import javax.xml.datatype.DatatypeConfigurationException;
import org.apache.jena.rdf.model.Model;
import org.apache.jena.riot.RDFDataMgr;
import org.apache.jena.riot.RDFFormat;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
import org.eclipse.lyo.oslc4j.core.model.IResource;
import org.eclipse.lyo.oslc4j.core.model.Link;
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper;

/**
 * Created on 2018-02-16
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public class ProblemBuilder {

    private final Problem problem;

    public ProblemBuilder() throws URISyntaxException {
        problem = new Problem();
    }

    public Problem buildProblem(final String problemUri) throws URISyntaxException {
        // no need to do this on Local resource
        problem.setAbout(new URI(problemUri));

        problem.setLabel("problem1");

        // FIXME Andrew@2018-02-15: Generate Goal resource class
//        Goal goal = new Goal();
//        problem.setGoal(new Link(goal.getAbout()));

        // FIXME Andrew@2018-02-15: Generate Init resource class
//        new Init()
        final HashSet<Link> init = new HashSet<Link>();
        problem.setInit(init);

        final Domain domain = buildDomain();
        problem.setDomain(domain);

        // TODO Andrew@2018-02-16: Create a Function instance
        // FIXME Andrew@2018-02-16: Generate total-time resource class
        final Function maxFunction = new Function();
        problem.setMaximize(new Link(maxFunction.getAbout()));

        // TODO Andrew@2018-02-16: check with Leo if any different from maximize
//        problem.setMinimize();

        final HashSet<PddlObject> pddlObjects = buildPddlObjects();
        problem.setPddlObject(pddlObjects);

        // FIXME Andrew@2018-02-16: it should be a link, no?
        final Collection<URI> types = new HashSet<URI>();
        // TODO Andrew@2018-02-16: create links that point to PrimitiveType instances
        final PrimitiveType location = new PrimitiveType();
        location.setLabel("location");
        final PrimitiveType block = new PrimitiveType();
        block.setLabel("block");

        problem.setTypes(types);


        return problem;
    }

    private HashSet<PddlObject> buildPddlObjects() throws URISyntaxException {
        final HashSet<PddlObject> pddlObjects = new HashSet<PddlObject>();
        final PddlObject pddlObject = new PddlObject();
        pddlObjects.add(pddlObject);
        return pddlObjects;
    }

    private Domain buildDomain() throws URISyntaxException {
        final Domain domain = new Domain();
        domain.setLabel("domain1");
        return domain;
    }

    static String serialiseToTurtle(final IResource problem)
            throws InvocationTargetException, DatatypeConfigurationException,
            OslcCoreApplicationException, IllegalAccessException {
        final StringWriter stringWriter = new StringWriter();
        final Model jenaModel = JenaModelHelper.createJenaModel(new Object[]{problem});
        RDFDataMgr.write(stringWriter, jenaModel, RDFFormat.TURTLE_PRETTY);
        return stringWriter.toString();
    }
}
