import eu.scott.warehouse.domains.pddl.Plan;
import eu.scott.warehouse.domains.pddl.Problem;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.net.URISyntaxException;
import javax.xml.datatype.DatatypeConfigurationException;
import net.oauth.OAuthException;
import org.apache.jena.rdf.model.Model;
import org.apache.wink.client.ClientResponse;
import org.eclipse.lyo.client.oslc.OslcClient;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper;

/**
 * Created on 2018-02-15
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public class Main {
    public static void main(String[] args)
            throws URISyntaxException, InvocationTargetException, DatatypeConfigurationException,
            OslcCoreApplicationException, IllegalAccessException, IOException, OAuthException {
        final ProblemBuilder problemBuilder = new ProblemBuilder();
        final Problem problem = problemBuilder.buildProblem("");

        final Model model = JenaModelHelper.createJenaModel(new Object[]{problem});

        final String turtle = ProblemBuilder.serialiseToTurtle(problem);

        System.out.println(turtle);

        // TODO Andrew@2018-02-16: uncomment the line below to try sending it to Planner Reasoner
//        createPlan(problem);
    }

    public static Plan createPlan(final Problem problem)
            throws IOException, OAuthException, URISyntaxException {
        final OslcClient client = new OslcClient();
        final ClientResponse resource = client.createResource(
                "http://aide.md.kth.se:3020/planner/planCreationFactory", new Object[]{problem},
                "text/turtle", "text/turtle");
        return resource.getEntity(Plan.class);
    }

}
