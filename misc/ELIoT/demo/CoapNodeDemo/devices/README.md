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
