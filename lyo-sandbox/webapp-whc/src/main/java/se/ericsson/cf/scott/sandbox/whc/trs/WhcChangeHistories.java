package se.ericsson.cf.scott.sandbox.whc.trs;

import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import javax.servlet.http.HttpServletRequest;
import org.eclipse.lyo.oslc4j.core.OSLC4JUtils;
import org.eclipse.lyo.oslc4j.core.model.IResource;
import org.eclipse.lyo.oslc4j.trs.provider.ChangeHistories;
import org.eclipse.lyo.oslc4j.trs.provider.HistoryData;

/**
 * Created on 2018-02-26
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public class WhcChangeHistories extends ChangeHistories {

    // TODO Andrew@2018-02-26: extract a default "manual" implementation
    public static final WhcChangeHistories INSTANCE = new WhcChangeHistories();
    private final List<HistoryData> history = new ArrayList<>();

    private WhcChangeHistories() {
        // FIXME Andrew@2018-02-27: the WhcTrsService shall pass the base on all requests
        this.setServiceBase(OSLC4JUtils.getServletURI());
    }

    @Override
    public HistoryData[] getHistory(final HttpServletRequest httpServletRequest, final Date dateAfter) {
        // TODO Andrew@2018-02-26: less expensive implementation
        // TODO Andrew@2018-02-26: consider switching the caller to use lists
        return history.toArray(new HistoryData[0]);
    }

    public void addResource(IResource resource) {
        final Date now = new Date();
        final HistoryData historyData = HistoryData.getInstance(now, resource.getAbout(), HistoryData.CREATED);
        history.add(historyData);
    }
}
