# SCOTT Twin Gateway

The concept of the Gateway is quite simple:

> Remove the need for the Device Experts to implement OSLC servers or need RDF
> libraries.

The Gateway is split into the **Gateway Frontend** and the **Gateway Backend**.
The Frontend communicates with the Robot via any proprietary protocol. The
Backend communicates with the Twins and the rest of the sandbox via REST/RDF
(over OSLC and TRS, possibly TRS over MQTT). Frontend and Backend communicate
with each other through an IPC mechanism exchanging a simple messages of fixed
format (msgpack over ZeroMQ currently).

## Getting started

* Frontend is generating most messages in this prototype, written in Python.
* Backend is a command line Kotlin app that dynamically loads (un)marshalling
  providers via classpath (`svc-sample` is has a sample service; implements
  interfaces defined in the `api`).

To run this setup:

1. Create a Python3 powered virtualenv environment for the frontend, install
   pip dependencies and run.
1. Install the `gateway-api` into your local Maven repository.
1. Package `gateway-svc-sample` as JAR and put it on the classpath of the
   backend
   ([Intellij](https://stackoverflow.com/questions/854264/how-to-add-directory-to-classpath-in-an-application-run-profile-in-intellij-idea))
1. Run the frontend and backend.

You should see messages being exchanged. The backend does both live unmarshalling to annotation-less Kotlin data classes as well as to the OSLC resources over a dynamically loaded `GatewayDiscoveryService`.

## Message format

**To be implemented**. Currently, the whole message is one arbitrary `msgBody`.

| Name        | Type                | Description                                    |
|-------------|---------------------|------------------------------------------------|
| `uuid`      | `str8`              | Unique ID, mostly for debugging                |
| `timestamp` | `timestamp64`       | Timestamp (as close to the msgBody generation) |
| `src`       | `str8`              | Source device ID (Gateway specific)            |
| `msgId`     | `str8`              | Message type ID                                |
| `msgBody`   | `bin32`             | Message body                                   |
| `metadata`  | `map16<str8,str16>` | Metadata                                       |

