package eu.scottproject.wp10.gw.svc.sample;

import com.fasterxml.jackson.annotation.JsonProperty;
import org.eclipse.lyo.oslc4j.core.model.AbstractResource;

/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
 */
public class WorkerMessageResource extends AbstractResource {
    @Override
    public String toString() {
        return "WorkerMessageResource{" + "number=" + number + '}';
    }

    private int number;

    // FIXME Andrew@2018-10-23: add OSLC annotations
    @JsonProperty("num")
    public int getNumber() {
        return number;
    }

    public void setNumber(final int number) {
        this.number = number;
    }
}
