package se.ericsson.cf.scott.sandbox.resources;

import java.net.URISyntaxException;
import java.util.Collection;
import java.util.HashSet;
import java.util.Random;
import java.util.stream.Collectors;
import org.eclipse.lyo.oslc4j.core.model.AbstractResource;
import org.eclipse.lyo.oslc4j.core.model.Link;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.WarehouseAdaptorManager;

/**
 * Created on 2017-06-20
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public class ResourceFactory {
    private final static Logger log = LoggerFactory.getLogger(ResourceFactory.class);
    // Use the system random seed
    private final static Random random = new Random(Double.doubleToLongBits(Math.random()));
    private final String spId;

    public ResourceFactory(String spId) {
        this.spId = spId;
    }

    public ResourceFactory() {
        this(WarehouseAdaptorManager.spId);
    }

    public WhObject buildObject(final String id, final String type, final Link place) {
        try {
            final WhObject object = new WhObject(spId, id);
            object.setType(type);
            object.setIsOn(place);
            return object;
        } catch (URISyntaxException e) {
            log.error("URI exception, check the spId string and parameters", e);
            return null;
        }
    }

    public WhObject buildObject(final String id, final String type, final Link place,
            int capacity) {
        final WhObject whObject = buildObject(id, type, place);
        whObject.setCapacity(capacity);
        return whObject;
    }

    public Place buildPlace(final String id, final String type, final Link waypoint) {
        try {
            final Place place = new Place(spId, id);
            place.setType(type);
            place.setSituatedAt(waypoint);
            place.setIsChargingStation("ChargingStation".equals(type));
            return place;
        } catch (URISyntaxException e) {
            log.error("URI exception, check the spId string and parameters", e);
            return null;
        }
    }

    public Place buildPlace(final String id, final String type, final Link waypoint,
            final int capacity) {
        try {
            final Place place = new Place(spId, id);
            place.setType(type);
            place.setSituatedAt(waypoint);
            place.setIsChargingStation("ChargingStation".equals(type));
            place.setCapacity(capacity);
            return place;
        } catch (URISyntaxException e) {
            log.error("URI exception, check the spId string and parameters", e);
            return null;
        }
    }

    public Waypoint buildWaypointDirect(final String id,
            final Collection<AbstractResource> canMove) {
        return buildWaypoint(id, collectionLinks(canMove));
    }

    public Waypoint buildWaypoint(final String id, final Collection<Link> canMove) {
        try {
            final Waypoint waypoint = new Waypoint(spId, id);
            waypoint.setCanMove(hashSetify(canMove));
            return waypoint;
        } catch (URISyntaxException e) {
            log.error("URI exception, check the spId string and parameters", e);
            return null;
        }
    }

    public Link l(AbstractResource r) {
        return new Link(r.getAbout());
    }

    public Link wayPointLink(final String id) {
        return Waypoint.constructLink(spId, id);
    }

    public Link placeLink(final String id) {
        return Place.constructLink(spId, id);
    }

    public String randomId() {
        return String.format("%x", random.nextInt());
    }

    public Robot buildRobot(final String robotId, final Link location, final int chargeLevel,
            final int capacity, final int maxCharge, final int highCharge, final int lowCharge,
            final boolean isCharging) {
        try {
            final Robot robot = new Robot(spId, robotId);
            robot.setIsAt(location);
            robot.setCapacity(capacity);
            robot.setChargeLevel(chargeLevel);
            robot.setMaxCharge(maxCharge);
            robot.setHighCharge(highCharge);
            robot.setLowCharge(lowCharge);
            robot.setIsCharging(isCharging);
            return robot;
        } catch (URISyntaxException e) {
            log.error("URI exception, check the spId string and parameters", e);
            return null;
        }
    }

    private HashSet<Link> linkSet(final Collection<? extends AbstractResource> canMove) {
        final Collection<Link> collectionLinks = collectionLinks(canMove);
        return hashSetify(collectionLinks);
    }

    private HashSet<Link> hashSetify(final Collection<Link> collectionLinks) {
        final HashSet<Link> links = new HashSet<>();
        links.addAll(collectionLinks);
        return links;
    }

    private Collection<Link> collectionLinks(final Collection<? extends AbstractResource> canMove) {
        return canMove.stream().map(o -> new Link(o.getAbout())).collect(Collectors.toList());
    }
}
