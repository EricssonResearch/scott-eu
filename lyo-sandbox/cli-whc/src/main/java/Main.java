import eu.scott.warehouse.domains.pddl.Plan;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.StringWriter;
import java.lang.reflect.InvocationTargetException;
import java.net.URISyntaxException;
import javax.xml.datatype.DatatypeConfigurationException;
import net.oauth.OAuthException;
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
import org.apache.wink.client.ClientResponse;
import org.eclipse.lyo.client.oslc.OslcClient;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Created on 2018-02-15
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public class Main {

    private final static Logger log = LoggerFactory.getLogger(Main.class);

    public static void main(String[] args) {

        final Model problemModel = loadJenaModelFromResource("sample-problem-request.ttl",
                Lang.TURTLE);

        Model plan = planForProblem(problemModel);

    }

    /**
     * TODO Andrew@2018-02-22: Not used now, switch to it later
     */
    public static Plan createPlan(final Object[] problem)
            throws IOException, OAuthException, URISyntaxException {
        final OslcClient client = new OslcClient();
        final ClientResponse resource = client.createResource(
                "http://aide.md.kth.se:3020/planner/planCreationFactory", problem, "text/turtle",
                "text/turtle");
        return resource.getEntity(Plan.class);
    }

    private static Model loadJenaModelFromResource(final String resourceName, final Lang lang) {
        final InputStream resourceAsStream = Main.class.getClassLoader().getResourceAsStream(
                resourceName);
        final Model problemModel = ModelFactory.createDefaultModel();
        RDFDataMgr.read(problemModel, resourceAsStream, lang);
        return problemModel;
    }

    private static Model planForProblem(final Model problemModel) {
        log.info("Problem request\n{}", jenaModelToString(problemModel));
        try {
            final InputStream response = requestPlanManually(problemModel);
            final Model responsePlan = ModelFactory.createDefaultModel();
            RDFDataMgr.read(responsePlan, response, Lang.TURTLE);
            log.info("Plan response\n{}", jenaModelToString(responsePlan));
            return responsePlan;
        } catch (IOException e) {
            log.error("Something went wrong", e);
            throw new IllegalStateException(e);
        }
    }

    private static InputStream requestPlanManually(final Model problemModel) throws IOException {
        String url = "http://aide.md.kth.se:3020/planner/planCreationFactory";

        final ByteArrayOutputStream out = new ByteArrayOutputStream();
        RDFDataMgr.write(out, problemModel, RDFFormat.TURTLE_BLOCKS);

        HttpClient client = HttpClientBuilder.create().build();
        HttpPost post = new HttpPost(url);

        post.setHeader("Content-type", "text/turtle");
        post.setHeader("Accept", "text/turtle");
        post.setEntity(new ByteArrayEntity(out.toByteArray()));

        HttpResponse response = client.execute(post);
        return response.getEntity().getContent();
    }

    private static String jenaModelToString(final Model responsePlan) {
        final StringWriter stringWriter = new StringWriter();
        RDFDataMgr.write(stringWriter, responsePlan, RDFFormat.TURTLE_PRETTY);
        return stringWriter.toString();
    }

    /**
     * TODO Andrew@2018-02-22: Not used now, switch to it later
     */
    private static Object[] buildProblem()
            throws URISyntaxException, DatatypeConfigurationException, IllegalAccessException,
            InvocationTargetException, OslcCoreApplicationException {
        final ProblemBuilder problemBuilder = new ProblemBuilder();
        final Object[] problem = problemBuilder.buildProblem("http://example.com/scott-sandbox");

        final Model model = JenaModelHelper.createJenaModel(problem);
        return problem;
    }

}
