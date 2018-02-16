import eu.scott.warehouse.domains.pddl.Problem;
import java.io.StringWriter;
import java.lang.reflect.InvocationTargetException;
import java.net.URISyntaxException;
import javax.xml.datatype.DatatypeConfigurationException;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
import org.junit.jupiter.api.Test;

import static org.assertj.core.api.Assertions.*;

/**
 * Created on 2018-02-16
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
class ProblemBuilderTest {

    @Test
    void buildProblem()
            throws URISyntaxException, InvocationTargetException, DatatypeConfigurationException,
            OslcCoreApplicationException, IllegalAccessException {
        final ProblemBuilder problemBuilder = new ProblemBuilder();

        final Problem problem = problemBuilder.buildProblem("urn:problem1");

        final String problemTurtleRepresentation = ProblemBuilder.serialiseToTurtle(problem);
        System.out.println(problemTurtleRepresentation);
        assertThat(problemTurtleRepresentation).contains("urn:problem1");
    }
}
