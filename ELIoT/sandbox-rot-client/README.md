## Getting started

    docker build -t sandbox-rot-client .


---

# Rot.js with Node-Coap

This is an simulation example based on [ELIOT](https://github.com/Alliasd/ELIoT) Platform. Through implementing [LWM2M](http://openmobilealliance.org/iot/lightweight-m2m-lwm2m) communication, robot node can register and update its location through respective SmartObject, further development for more use case is ongoing.

## Prerequisite

* Docker (Version 17 or newer).
* Leshan 0.1.11 M5 SNAPSHOT or 0.1.11 M4
* npm
* JsDoc (For developers)

## Usage

Client:

```shell
./run.sh
```

Server:

```shell
docker build -t server .
docker run --rm -ti --name mi server
```

## Client Developing Guide

Client is devepled based on `Coap-node` project from npm in Node.js. The communication flow is designed according to [RFC CoAP](https://tools.ietf.org/html/rfc725)  in lower network layers. LWM2M is kept as application layer protocol. SmartObject is employed as the data model of the client.

[IPSO](https://www.ipso-alliance.org/smart-object-guidelines/) defines a hierarchical data model to describe real-world gadgets, such as temperature sensors and light controllers, which is called [SmartObject](https://www.npmjs.com/package/smartobject) . In our use case, each smart object will be simulated node with unique IP address and registration ID. To illustrate the data model:

* Rot

  * Object: Server :  ``oid = 1``

    * Instance : Server Tom : ``iid = 0``
    * Instance : Server Jack : ` iid = 1`
      * Resource: _Server ID : ` rid = 0 ` (Protected resource, user can not change after initialized)
      * Resource: Connection Lifetime: ` rid = 1 `

  * Object: Device : ``oid = 2``

    …….

  * Object: Temptature sensor `` oid = 3303 ``

The set object ID could be string or integer, but it has to be included at [list of oids](https://github.com/simenkid/lwm2m-id#Identifiers). The resources are abstract of hardware KPIs, a suggested [Resources Planning Tutorial](https://github.com/AllSmartObjects/smartobject/blob/master/docs/resource_plan.md) and the [Code Template](https://github.com/AllSmartObjects/smartobject/blob/master/docs/templates.md) is given.

When constructing a new SmartObject, a hands-on approach could be:

````javascript
// Import the class
var SmartObject = require('smartobject');
var so = new SmartObject();

// Initialized the object
so.init('temperature', 0, {
    sensorValue: 31,
    units : 'C'
});

// Read a specific value from the source
so.read('temperature', 1, 'sensorValue', function (err, data) {
    if (!err)
        console.log(data);
});

// Write a value to a resource
so.write('temperature', 1, 'sensorValue', 26, function (err, data) {
    if (err) {
        console.log(err);
    }
});
````

In terms of detailed API documentation, please refer to npm [SmartObject Doc](https://www.npmjs.com/package/smartobject#API_smartobject).

For various use case, user may need to let client initiate a request. In such case, user can reference [Coap-node](https://github.com/mcollina/node-coap) library, this lib is also adapted by Node-coap in sending CoAP-ping and registration information to the server.

---


# Emulated IoT (ELIOT) Platform

The Emulated IoT (ELIOT) platform enables emulating simple IoT devices on Docker. It is based on libraries, [coap-node](https://github.com/PeterEB/coap-node) and [Leshan](https://github.com/eclipse/leshan), which implement device management over CoAP and LWM2M. Devices consisting of simple sensors and actuators can be built using IPSO objects. The current implementation provides ready-to-use devices, such as weather observer, presence detector, light controller and radiator.

More detailed information about ELIOT can be found from [wiki](https://github.com/Alliasd/ELIOT/wiki)

**This module implements the following LWM2M interfaces & operations:**


| Bootstrap | Register   | Device Management & Service Enablement | Information Reporting |
| --------- | ---------- | -------------------------------------- | ---------------------
| Bootstrap | Register   | Read (Text/JSON/TLV)  | Observe (Text/JSON/TLV) |
|           | Update     | Write (Text/JSON/TLV) | Notify (Text/JSON/TLV)  |
|           | Deregister | Create (JSON/TLV)     | Cancel                  |
|           |            | Execute               |                         |
|           |            | Delete                |                         |
|           |            | Write attributes      |                         |


## Usage with Docker

1. Run the LWM2M Server:

   `docker run --rm -ti --name ms corfr/leshan `

2. Run the LWM2M Bootstrap Server:

   `docker run --rm -ti --name bss --link ms corfr/leshan bootstrap`

3. Run the devices (PresenceDetector | WeatherObserver | LightController | Radiator):

   **Without Bootstrap Server**

   `docker run -it --link ms alliasd/eliot <jsfile_name> ms [OPTIONS]`

   **With Bootstrap Server**

   `docker run -it --link bss --link ms alliasd/eliot <jsfile_name> bss [OPTIONS]`

   ```
   JS-files:
   * presence.js
   * weather.js
   * light_control.js
   * radiator.js

   Options:
   -b          Bootstrap mode
   -t          Real Weather data (only WeatherObserver)
   ```

   **Note**: instead of using the name ms/bss you can use the IP address without the --link flags.

4. Run multiple clients with docker-compose

   `docker-compose up`

   `docker-compose scale weather=X presence=X radiator=X light=X`
