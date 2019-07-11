import eu.scott.warehouse.domains.blocksworld.Block;
import eu.scott.warehouse.domains.blocksworld.Location;
import eu.scott.warehouse.domains.blocksworld.On;
import eu.scott.warehouse.domains.pddl.Domain;
import eu.scott.warehouse.domains.pddl.Function;
import eu.scott.warehouse.domains.pddl.Or;
import eu.scott.warehouse.domains.pddl.PddlObject;
import eu.scott.warehouse.domains.pddl.Problem;
import java.io.StringWriter;
import java.lang.reflect.InvocationTargetException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.stream.Collectors;
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
    private String base;

    public ProblemBuilder() throws URISyntaxException {
        problem = new Problem();
    }

    public Object[] buildProblem(final String base) throws URISyntaxException {
        this.base = base;

        // no need to do this on Local resource
        problem.setAbout(relUri("problem1"));
        problem.setLabel("problem1");

//         FIXME Andrew@2018-02-15: Generate Init resource class
//        new Init()
        final HashSet<Link> init = new HashSet<Link>();

        problem.setInit(init);

        final Domain domain = buildDomain();
        problem.setDomain(linkTo(domain));

        // TODO Andrew@2018-02-16: Create a Function instance
        // FIXME Andrew@2018-02-16: Generate total-time resource class
        final Function maxFunction = new Function(relUri("total-time"));
        problem.setMinimize(linkTo(maxFunction));

        // TODO Andrew@2018-02-16: check with Leo if any different from maximize
//        problem.setMinimize();

        final HashSet<PddlObject> pddlObjects = buildPddlObjects();
        problem.setPddlObject(linkTo(pddlObjects));

        // FIXME Andrew@2018-02-16: it should be a link, no?
        // TODO Andrew@2018-02-16: create links that point to PrimitiveType instances
        final Location table = new Location(relUri("table"));
        table.setLabel("table");
        final Block a = new Block(relUri("a"));
        a.setLabel("a");
        final Block b = new Block(relUri("b"));
        b.setLabel("b");
        final Block c = new Block(relUri("c"));
        c.setLabel("c");

        problem.setPddlObject(linkTo(Arrays.asList(a, b, c, table)));

        // FIXME Andrew@2018-02-15: Generate Goal resource class
//        Goal goal = new Goal();
//        problem.setGoal(new Link(goal.getAbout()));
        final Or or = new Or(relUri("localOr"));

        final On on1 = new On(relUri("localOn1"));
        on1.setOnX(linkTo(b));
        on1.setOnY(linkTo(c));

        final On on2 = new On(relUri("localOn2"));
        on2.setOnX(linkTo(c));
        on2.setOnY(linkTo(b));

        or.addArgument(linkTo(on1));
        or.addArgument(linkTo(on2));
        problem.setGoal(linkTo(or));

        return new Object[]{problem, domain, maxFunction, pddlObjects, a, b, c, table, or, on1,
                on2};
    }

    // FIXME Andrew@2018-02-16: Jad, this should go as a static Link.to() method
    public static Link linkTo(IResource resource) {
        return new Link(resource.getAbout());
    }

    // FIXME Andrew@2018-02-16: Jad, this should go as a static Link.to() method
    public static HashSet<Link> linkTo(Collection<? extends IResource> resource) {
        return resource.stream().map(ProblemBuilder::linkTo).distinct().collect(
                Collectors.toCollection(HashSet::new));
    }

    private URI relUri(final String relativeUriSegment) throws URISyntaxException {
        return new URI(base + relativeUriSegment);
    }

    private HashSet<PddlObject> buildPddlObjects() throws URISyntaxException {
        final HashSet<PddlObject> pddlObjects = new HashSet<PddlObject>();
        final PddlObject pddlObject = new PddlObject();
        pddlObjects.add(pddlObject);
        return pddlObjects;
    }

    private Domain buildDomain() throws URISyntaxException {
        final Domain domain = new Domain(relUri("adl-blocksworld"));
        domain.setLabel("adl-blocksworld");
        return domain;
    }

    static String serialiseToTurtle(final IResource problem)
            throws InvocationTargetException, DatatypeConfigurationException,
            OslcCoreApplicationException, IllegalAccessException {
        return serialiseToTurtle(new Object[]{problem});
    }

    static String serialiseToTurtle(final Object[] resources)
            throws InvocationTargetException, DatatypeConfigurationException,
            OslcCoreApplicationException, IllegalAccessException {
        final StringWriter stringWriter = new StringWriter();
        final Model jenaModel = JenaModelHelper.createJenaModel(resources);
        RDFDataMgr.write(stringWriter, jenaModel, RDFFormat.TURTLE_PRETTY);
        return stringWriter.toString();
    }
}
