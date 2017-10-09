# Extensions to the OSLC TRS 2.0 specification

## Motivation

- eliminate polling
- order TRS change events related PDDL actions sequentially
- apply TRS change events for a complex PDDL effect atomically

## Proposed modifications

### Polling elimination

Instead of appending the change events to a TRS change log that can be fetched via a REST call, we will publish each change event to an MQTT channel. Effectively, each OSLC microservice will host a TRS server and a TRS client.

### Sequential change event ordering


To allow multiple resources to be
updated atomically, the _change events_ in our architecture were extended with
the following properties, borrowed from the Message Sequence pattern
\cite[ch.~5, p.~170]{hohpe2003enterprise}:

- sequence number `oscl_trs:sequence` of integer type, in order to distinguish
  different sequences;

- event position `oscl_trs:position` of integer type, to strictly order the
  events in the sequence;

- end flag `oscl_trs:end` of boolean type, to mark the end of the sequence.

These changes allow sequence buffers to be maintained on the _client_. The
_client_ would then accumulate the incoming _change events_ that are part of the
same sequence in the buffer in the correct order until the flagged _change
event_ arrives as well as all the _change events_ in the preceding positions.

<div style="text-align:center">***</div>

Typically, a TRS _server_ publishes a _change event_ that contains a metadata
about the change, such as whether it was a modification, deletion, or creation;
the URI of the resource; etc. The resource state at the moment of the event
occurrence is, however, not included in the event and the TRS _client_ has to
fetch via a separate REST call. Such approach is problematic in our use-case,
since the state of the resource may well have changed between the time the
original event was produced and the time the TRS _client_ requests the resource
state. In this case, resource updates might effectively be processed
out-of-order, even if a strict order of the events is maintained.

Our architecture remedies it by embedding the full state of the tracked resource
in the corresponding _change event_ so that no other factors can influence the
event ordering. An additional benefit of embedding is that it saves the TRS
client from making an additional HTTP request to fetch the resource itself. Push
approach as well as reduced number of requests may help to reduce the overall
system load and minimise the delays caused by polling.

## More motivation

Leo has discovered an issue is that a single PDDL action may have an effect on different resources managed by different adaptors. Therefore, an atomic application of the TRS events across multiple TRS change logs is necessary.

Furthermore, we might need to see a consistent state across the whole sandbox and strictly sequential atomic change event application is not enough. A simple example is the case when a robot unloads a box onto a shelf. The states of the Box, Robot, and Shelf resources are changed atomically and the TRS change event sequence has been published. The problem is that at a certain point of time, a fraction of the subscribers has applied this change event sequence atomically, while the rest of the subscribers have not. We might need to avoid that by gathering a disributed snapshot

## More crazy changes :fire:

One way to fix the atomic TRS change event application across multiple change logs is to generate action UUIDs and use them as sequence IDs while relying on vector clock during the publishing of the TRS events to generate monotonically increasing position ids. The total number of the events in a sequence needs to be determined independently (ie needs to be known beforehand, eg by counting the total number of predicates in the event).

In order to construct a distributed snapshot on top of TRS, a new TRS _token resource_ is introduced.

TRS _token_ resource `oslc_trs:Token` contains the following properties:

- URI, as every RDF resource has;
- RDF type, as every OSLC resource must have one;
- the sender;
- the receiver;
- timestamp, which is optional and not reliable due to the problem of distributed clocks;
