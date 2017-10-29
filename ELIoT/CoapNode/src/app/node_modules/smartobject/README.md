# smartobject
A Smart Object Class that helps you with creating IPSO Smart Objects in your JavaScript applications

[![NPM](https://nodei.co/npm/smartobject.png?downloads=true)](https://nodei.co/npm/smartobject/)  

[![Travis branch](https://img.shields.io/travis/PeterEB/smartobject/master.svg?maxAge=2592000)](https://travis-ci.org/AllSmartObjects/smartobject)
[![npm](https://img.shields.io/npm/v/smartobject.svg?maxAge=2592000)](https://www.npmjs.com/package/smartobject)
[![npm](https://img.shields.io/npm/l/smartobject.svg?maxAge=2592000)](https://www.npmjs.com/package/smartobject)

## Table of Contents

1. [Overview](#Overview)  
2. [Installation](#Installation)  
3. [Usage](#Usage)  
4. [Resources Planning](#Resources)  
5. [APIs](#APIs)  
6. [Code Templates](https://github.com/AllSmartObjects/smartobject/blob/master/docs/templates.md)  

<a name="Overview"></a>  
## 1. Overview

**smartobject** is a _Smart Object_ Class that helps you with creating [_IPSO_](http://www.ipso-alliance.org/) _Smart Objects_ in your JavaScript applications. If you like to use the IPSO data model in your projects or products, you can use **smartobject** as the base class to abstract your hardware, sensor modules, or gadgets into plugins (node.js packages) for users convenience. Here is [an example of hardware abstraction with mraa](https://github.com/AllSmartObjects/smartobject/wiki/Hardware-Abstraction-with-mraa) on Linkit Smart 7688 in our wiki. In addition, this module is isomorphic and you can use it at server-side as well to generate the smart objects.  
  
IPSO defines a hierarchical data model to describe real-world gadgets, such as temperature sensors and light controllers.  
* IPSO uses _**Object**_ to tell what kind of a gadget is, and uses _**Object Instance**_ to tell which one a gadget is.  
* An _**Object**_ is like a class or a boilerplate, and each kind of _Object_ has an unique _Object Id_ (`oid`) defined by IPSO, e.g., 3303 for the _Temperature Sensor Object_. Here is the [list of oids](https://github.com/simenkid/lwm2m-id#Identifiers).  
* An _**Object Instance**_ is the entity of an _Object_. Each _**Object Instance**_ has an unique _**Object Instance Id**_ to identify itself from other gadgets of the same class. Simply speaking, `oid` is like a namespace to manage all the same kind of _IPSO Object Instances_.  
* The _**Resources**_ are used to describe what attributes may a gadget have, for example, a temperature sensor may have attributes such as _sensorValue_, _unit_, _minMeaValue_, .etc. _**Resource Values**_  will be filled after instantiated. Here is the [list of templates](https://github.com/AllSmartObjects/smartobject/blob/master/docs/templates.md) to show you what attributes may a gadget have.  

![ISPO Model](https://raw.githubusercontent.com/lwmqn/documents/master/media/ipso_model.png)


[**Note**]
* The _italics_, such _Object_, _Object Id_, _Object Instance_, and _Object Instance Id_, are used to distinguish the _**IPSO Objects**_ from the JavaScript **objects**.  
  

<a name="Installation"></a>  
## 2. Installation

> $ npm install smartobject --save

<a name="Usage"></a>  
## 3. Usage

Here is a quick example to show you how to create your _Smart Object_ with only few steps:

* **Step 1**: Import the SmartObject Class and create an instance from it 
    ```js
    var SmartObject = require('smartobject');
    var so = new SmartObject(); // so can hold many Object Instances in it
    ```

* **Step 2**: Initialize a temperature sensor in your smart object `so`  (ref: [code templates](https://github.com/AllSmartObjects/smartobject/blob/master/docs/templates.md))
    ```js
    so.init(
        'temperature',          // 'temperature' is the IPSO-defined Object Identifier (oid, 3303).
        0,                      // 0 is the unique Object Instance Id (iid) assigned by you.
        {                       // This object contains all Resources (attributes) this sensor has. 
            sensorValue: 31,    // 'sensorValue' is the IPSO-defined Resource Id (rid, 5700) 
            units : 'C'         // 'units' is the IPSO-defined Resource Id (rid, 5701) 
        }
    );
    ```
* **Step 3**: Initialize more _Object Instances_. Finally, we have 3 temperature sensors, 1 magnetometer, and 4 digital inputs in our `so`
    ```js
    // Init more temperature sensors (each with an unique iid)
    so.init('temperature', 1, {
        sensorValue: 28,
        units : 'C'
    });

    so.init('temperature', 2, {
        sensorValue: 72.6,
        units : 'F'
    });

    // Init other gadgets
    so.init('magnetometer', 0, {
        xValue: 18,
        yValue: 21,
        zValue: 231
    });

    so.init('dIn', 0, { dInState: 1 });
    so.init('dIn', 1, { dInState: 0 });
    so.init('dIn', 6, { dInState: 0 });
    so.init('dIn', 7, {
        // if dInState should be read from by certain operation
        dInState: {
            read: function (cb) {
                var hal = this.parent.hal;  // see SmartObject constructor
                hal.digitalPin0.read(function (err, val) {
                    cb(null, val);
                });
            }
        }
    });
    ```

<a name="Resources"></a>  
## 4. Resources Planning

The great benefit of using **smartobject** in your application is that you almost need not to tackle the allocation of _Resources_ by yourself. It provides a scheme to help you with management of reading/writing your hardware or executing a procedure on the machine. All you have to do is to plan and define your _Resources_ well, and then use **smartobject** methods to do your jobs. You can use **smartobject** to abstract your hardware, sensor modules, or gadgets into plugins (node.js packages).  

Imagine that you have to read the temperature value from a sensor with one-wire interface:
* How to export this sensor to an IPSO smart object?
* How do you read the temperature value from your smart object?
* How do you do with the access control? Your _Resource_ is readable? writable? or remotely executable?  

Please refer to [Resources Planning Tutorial](https://github.com/AllSmartObjects/smartobject/blob/master/docs/resource_plan.md) for more details. It will show you how to initialize your _Resources_ and how to abstract your hardware with _IPSO Resources_ as well. In addition, here are some [code templates](https://github.com/AllSmartObjects/smartobject/blob/master/docs/templates.md) for your convenience to create smart objects.  

<a name="APIs"></a>  
## 5. APIs

* [new SmartObject()](#API_smartobject)
* [init()](#API_init), [remove()](#API_remove)
* [objectList()](#API_objectList)
* [has()](#API_has), [get()](#API_get), [set()](#API_set)
* [read()](#API_read), [write()](#API_write), [exec()](#API_exec)
* [dump()](#API_dump), [dumpSync()](#API_dumpSync)
* [isReadable()](#API_isReadable), [isWritable()](#API_isWritable), [isExecutable()](#API_isExecutable)

[**Note**]
* In general, the most often used APIs are `new SmartObject()`, `init()`, `read()`, and `write()`. It's not that complicated as it looks like.  
  
*************************************************
## SmartObject Class
Exposed by `require('smartobject')`.  

<a name="API_smartobject"></a>
### new SmartObject([hal][, setup])
Create an instance of SmartObject class. This document will use `so` to indicate this kind of instance.  

A `so` can hold many _IPSO Object Instances_ in it. The `so` itself has an accessible but un-enumerable boolean property `'ipsoOnly'` to define if this `so` only accepts IPSO-defined `oid` and `rid`. Default value for `so.ipsoOnly` is `false`. You can set it to `true` in the `setup` function.  

If `so.ipsoOnly == true`, then the given `oid` must be an IPSO-defined Object Id, `iid` must be a number, and all keys within `resrcs` object must be IPSO-defined Resource Ids, or `so.init()` will throw Errors.  

**Arguments:**  

1. `hal` (_Object_): Optional. A component or controller of the hardware abstraction layer. It will be assigned to `this.hal` at creation of a `so`. Noted that `so.hal` is accessible but un-enumerable.  
2. `setup` (_Function_): Optional. A setup function allows you to do some initializing work, for example, setting gpio direction. In the setup function, `this` will be bound to the `so` itself, thus you can use `this.hal` to access your hardware.  

**Returns:**  

* (_Object_): **so**  

**Examples:** 

* A very simple case. There is no hardware with the smart object. For example, at server-side we only need the **data** of a smart object, thus we don't have the `hal`.  

```js
var SmartObject = require('smartobject');

var so = new SmartObject();
```

* No hardware, and you like to make `so` accept only IPSO-defined identifiers.  

```js
var SmartObject = require('smartobject');

var so = new SmartObject(function () {
    this.ipsoOnly = true;
});
```

* We have 2 LEDs and 1 Switch controlled via `mraa`. This is a typical example at client-side (machine).  

```js
var m = require('mraa');
var SmartObject = require('smartobject');

var myHardware = {
    led1: new m.Gpio(44),
    led2: new m.Gpio(44),
    onOffSwitch: new m.Gpio(45),
    foo: 'bar'
};

var so = new SmartObject(myHardware, function () {
    var hal = this.hal;

    // hardware initialization
    hal.led1.dir(m.DIR_OUT);
    hal.led2.dir(m.DIR_OUT);
    hal.onOffSwitch.dir(m.DIR_IN);

    hal.foo = 'initialized';
    this.ipsoOnly = true;
});
```

*************************************************
<a name="API_init"></a>
### init(oid, iid, resrcs[, setup])
Create and initialize an _Object Instance_ in `so`, where `oid` is the [_IPSO Object Id_](https://github.com/simenkid/lwm2m-id#Identifiers) to indicate what kind of your gadget is, `iid` is the _Object Instance Id_, and `resrcs` is an object that wraps up all the _Resources_.  
  
* Be careful, invoking `init()` against an existing _Object Instance_ will firstly wipe out all its _Resources_ and inner `_state` and then put the new _Resources_ into it. Thus, it is better to initialize your _Instance_ only once throughout your code.  
* Property `_state` is a special _Resource_ that is an **accesible** but **un-enumerable** protected member in the _Object Instance_. It is an object where you can maintain some private information or inner state within the _Object Instance_. We will talk about it more later.  

**Arguments:**  

1. `oid` (_String_ | _Number_): _IPSO Object Id_, for example, `'temperature'` or `3303`. `so` will internally turn the id into its string version, say `'temperature'`, as the key if given with a numeric id.  
2. `iid` (_String_ | _Number_): _Object Instance Id_. It would be nice to use numbers, i.e., `0`, `1`, `2` to strictly meet the IPSO definition. But strings are also accepted, e.g., `'sen01'`, `'sen02'`, `'sen03'`, it is just like a handle to help you distinguish different _Instances_ that share the same _Object_ class.  
3. `resrcs` (_Object_): _IPSO Resources_, which is an object with **rid-value pairs** to describe the _Resources_. Each key in `resrcs` is a _Resource Id_ that can be a string or a number. And each value can be a primitive, an data object, or an object with specific methods, i.e. read(), write(), exec(). The [Resources Planning Tutorial](https://github.com/AllSmartObjects/smartobject/blob/master/docs/resource_plan.md) will give you some hints. You can have your private information or inner states within an object assigned to the `resrc._state` property, for example `resrc = { _state: { foo: 'bar' } }`.  
4. `setup` (_Function_): Optional. A setup function allows you to set some things up, for example, setting some flags or states for inner use. In this function, `this` will be bound to the _Object Instance_ itself, thus you can use `this._state` to access your inner state. Further more, you can use `this.parent` to get the `so` that owns this _Object Instance_, and use `this.parent.hal` to access your hardware.  

**Returns:**  

* (_Object_): `objInst`, the initialized _Object Instance_.  

**Examples:** 

* A very simple case. There is no hardware with the smart object.  

```js
var so = new SmartObject();

so.init('temperature', 0, {
    sensorValue: 31,
    units : 'C'
});

so.init('temperature', 1, {
    _state: {           // inner state
        foo: 'bar'
    },
    sensorValue: 75,
    units : 'F'
});

so.init(3303, 18, {
    sensorValue: 301,
    units : 'K'
}, function () {
    // this._state is an empty object by default
    // you can attach things to it
    this._state.foo = 'bar';
});

// Dumped data of the so will look like:
// (inner _state will not be dumped)
/*
{
    temperature: {
        '0': {
            sensorValue: 31,
            units : 'C'
        },
        '1': {
            sensorValue: 75,
            units : 'F'
        },
        '18': {
            sensorValue: 301,
            units : 'K'
        }
    }
}
*/
```

* We have 1 LED and 1 On/Off Switch controlled via `mraa`.

```js
var m = require('mraa');
var SmartObject = require('smartobject');

var so = new SmartObject({
    led: new m.Gpio(44),
    onOffSwitch: new m.Gpio(45)
}, function () {
    var hal = this.hal;

    // hardware initialization
    hal.led.dir(m.DIR_OUT);
    hal.onOffSwitch.dir(m.DIR_IN);

    this.ipsoOnly = true;
});

// led
so.init('lightCtrl', 0 , {
    _state: {   // protected resource to maintain inner states
        readCounts: 0,  // to record times of read
        writeCounts: 0  // to record times of written
    },
    onOff: {
        read: function (cb) {
            // 'this' is bound to Object Instance itself
            // this.parent === so
            var hal = this.parent.hal;
            var ledState = hal.led.read();

            this._state.readCounts += 1;    // inner record
            cb(null, ledState);
        },
        write: function (val, cb) {
            var hal = this.parent.hal;
            hal.led.write(val);

            this._state.writeCounts += 1;   // inner record
            cb(null, hal.led.read());
        }
    }
});
```

*************************************************
<a name="API_remove"></a>
### remove(oid, iid)
Remove an _Object Instance_ in `so`, where `oid` is the [_IPSO Object Id_](https://github.com/simenkid/lwm2m-id#Identifiers) to indicate what kind of your gadget is, `iid` is the _Object Instance Id_.

**Arguments:**  

1. `oid` (_String_ | _Number_): _IPSO Object Id_, for example, `'temperature'` or `3303`. `so` will internally turn the id into its string version, say `'temperature'`, as the key if given with a numeric id.  
2. `iid` (_String_ | _Number_): _Object Instance Id_. It would be nice to use numbers, i.e., `0`, `1`, `2` to strictly meet the IPSO definition. But strings are also accepted, e.g., `'sen01'`, `'sen02'`, `'sen03'`, it is just like a handle to help you distinguish different _Instances_ that share the same _Object_ class.  

**Returns:**  

* (_Boolean_): Returns `true` if remove successfully, else returns `false` if the _Object Instance_ does not exist.  

**Examples:** 

```js
so.remove('temperature', 0)
```

*************************************************
<a name="API_objectList"></a>
### objectList()
Returns the list of _Objects_ and _Object Instances_ with their identifiers. If an _Id_ is an IPSO-defined one, it will be returned as a number. If you're using LWM2M interface, you may need this method to generate the _Object List_ when registering to a server.  
  
**Arguments:**  

1. _none_  

**Returns:**  

* (_Array_): Returns an array that contains all the identifiers, each element is in the form of `{ oid: 3301, iid: [ 0, 1, 2, 3 ] }`.  

**Examples:** 

```js
var so = new SmartObject();

so.init('temperature', 0, {
    sensorValue: 31,
    units : 'C'
});

so.init('temperature', 18, {
    sensorValue: 301,
    units : 'K'
});

so.init('illuminance', 0, {
    sensorValue: 128.6
});

so.initResrc('presence', 6, {
    dInState: 0
});

so.initResrc('myGadget', 'gad72', {
    myResource: 'hello_world'
});

so.objectList();
/*
[
    { oid: 3303, iid: [ 0, 18 ] },
    { oid: 3301, iid: [ 0 ] },
    { oid: 3302, iid: [ 6 ] },
    { oid: 'myGadget', iid: [ 'gad72' ] }    // not IPSO-defined
]
*/
```

*************************************************
<a name="API_has"></a>
### has(oid[, iid[, rid]])
To see if `so` has the specified _Object_, _Object Instance_, or _Resource_.  

**Arguments:**  

1. `oid` (_String_ | _Number_): _Object Id_ of the target.  
2. `iid` (_String_ | _Number_): _Object Instance Id_ of the target.  
3. `rid` (_String_ | _Number_): _Resource Id_ of the target.  

**Returns:**  

* (_Boolean_): Returns `true` if target exists, otherwise `false`.  

**Examples:** 

```js
// Checks if so has the 'humidity' Object
so.has('humidity');                       // true

// Checks if so has the 'foo' Object Instance with iid = 0
so.has('foo', 0);                         // false

// Checks if so has the 'sensorValue' Resource in temperature sensor 8
so.has('temperature', 8, 'sensorValue');  // true
```

*************************************************
<a name="API_get"></a>
### get(oid, iid, rid)
Synchronously get the specified _Resource_.  

* At client-side (machine), the `get()` method is usually used to get the raw _Resource_ which may be an object with read/write/exec callbacks. If you like to read the **exact value** of a _Resource_, you should use the asynchronous `read()` method. Since reading something from somewhere may require some special and asynchronous operations, such as reading data from a wire, and reading from a database.  
* At server-side (data center), the _Resource values_ are simple data pieces requested from machines. Thus, using `get()` to get the _stored value of a Resource on the server_ is no problem.  

**Arguments:**  

1. `oid` (_String_ | _Number_): _Object Id_ of the target.  
2. `iid` (_String_ | _Number_): _Object Instance Id_ of the target.  
3. `rid` (_String_ | _Number_): _Resource Id_ of the target.   

**Returns:**  

* (_Depends_): Returns the _Resource_ value, or `undefined` if _Resource_ does not exist.  

**Examples:** 

```js
so.get('temperature', 2, 'sensorValue');  // 26.4

// If the Resource is an object with read/write/exec method(s)
so.get('temperature', 1, 'sensorValue');
/*
{
    read: function (cb) { ... }
}
*/

// If you do like to read the exact value from the temperature sensor, please use read()
so.read('temperature', 1, 'sensorValue', function (err, data) {
    if (!err)
        console.log(data);  // 18.4
});

```

*************************************************
<a name="API_set"></a>
### set(oid, iid, rid, value)
Synchronously set a value to the specified _Resource_.  

* At client-side (machine), the `set()` method is usually used to **initialize** a _Resource_, but not to write a value to a _Resource_. You should use the asynchronous `write()` method if you like to write a value to the _Resource_. Since writing something to somewhere may require some special and asynchronous operations, such as writing data to a wire, and writing data to a database.  
* At server-side (data center), use `set()` to _store the value of a Resource on the server_ is no problem. For example, when your request of reading a _Resource_ from a remote machine has responded back, you can use `set()` to store that _Resource value_ on the server.  

**Arguments:**  

1. `oid` (_String_ | _Number_): _Object Id_ of the target.  
2. `iid` (_String_ | _Number_): _Object Instance Id_ of the target.  
3. `rid` (_String_ | _Number_): _Resource Id_ of the target.   
4. `value` (_Primitives_ | _Object_): _Resource_ data or an object with read/write/exec method(s). This method will throw if `value` is given with a function.  

**Returns:**  

* (_Boolean_): Returns `true` if set successfully, else returns `false` if the _Object Instance_ does not exist (_Resource_ cannot be set).  

**Examples:** 

```js
so.set('dIn', 0, 'dInState', 1);    // true
so.set('dOut', 1, 'dOutState', 0);  // true

so.set('dOut', 2, 'dOutState', {
    read: function (cb) {
        gpioA3.read(function (state) {  // assume gpioA3 is a handle to your hardware
            cb(null, state);
        });
    }
});  // true

so.set('dOut', 2, 'dOutState', function (cb) {
    gpioA3.read(function (state) {
        cb(null, state);
    });
});  // throw Error, value cannot be a function
```

*************************************************
<a name="API_read"></a>
### read(oid, iid, rid[, opt], callback)
Asynchronously read the specified _Resource_ value.  

**Arguments:**  

1. `oid` (_String_ | _Number_): _Object Id_ of the target.
2. `iid` (_String_ | _Number_): _Object Instance Id_ of the target.
3. `rid` (_String_ | _Number_): _Resource Id_ of the target.
4. `opt` (_Object_): An option used to read _Resources_ in restrict mode, default is `{ restrict: false }`. If it is given with `{ restrict: true }`, this method will follow the access control specification defined by IPSO. This option may be set to `true` to respond to a remote _read request_ (access from outside world should be under control).
5. `callback` (_Function_): `function (err, data) { ... }`. Will be called when reading is done or any error occurs, where `data` is the _Resource_ value. (When an error occurs, `so` will pass you a string like `'_notfound_'` with `data`, you can use it as a hint to choose a status code to respond back to the requester.)
  
* This table show you what results may the callback receive:   

|       err      |      data        |       Description                                                  |  
|----------------|------------------|--------------------------------------------------------------------|  
| Error object   | `'_notfound_'`   | _Resource_ not found.                                              |  
| Error object   | `'_unreadable_'` | _Resource_ is unreadable.                                          |  
| Error object   | `'_exec_'`       | _Resource_ is unreadable (Because it is an executable _Resource_). |  
| `null`         | Depends          | _Resource_ is successfully read.                                   |  


**Returns:**  

* (_none_)  

**Examples:** 

```js
so.read('temperature', 1, 'sensorValue', function (err, data) {
    if (!err)
        console.log(data);  // 18.4
});

so.read('actuation', 0, 'dimmer', function (err, data) {
    if (!err)
        console.log(data);  // 62
});

so.read('illuminance', 1, 'maxMeaValue', function (err, data) {
    if (err) {
        console.log(err);   //  Error: 'Resource is unreadable.'
        console.log(data);  // '_unreadable_'
    }
});

so.read('accelerometer', 2, 'minRangeValue', function (err, data) {
    if (err) {
        console.log(err);   //  Error: 'Resource not found.'
        console.log(data);  // '_notfound_'
    }
});

so.read('barometer', 6, 'resetMinMaxMeaValues', function (err, data) {
    if (err) {
        console.log(err);   //  Error: 'Resource is unreadable.'
        console.log(data);  // '_exec_'
    }
});
```

*************************************************
<a name="API_write"></a>
### write(oid, iid, rid, value[, opt], callback)
Asynchronously write a value to the specified _Resource_.  

**Arguments:**  

1. `oid` (_String_ | _Number_): _Object Id_ of the target.  
2. `iid` (_String_ | _Number_): _Object Instance Id_ of the target.  
3. `rid` (_String_ | _Number_): _Resource Id_ of the target.   
4. `value` (_Depends_): The value to write to the specified _Resource_.  
5. `opt` (_Object_): An option used to write _Resources_ in restrict mode. Default is `{ restrict: false }` if not given.  
6. `callback` (_Function_): `function (err, data) { ... }`. Will be called when writing is done or any error occurs, where `data` is the _Resource_ value written. (When an error occurs, `so` will pass you a string like `'_notfound_'` with `data`, you can use it as a hint to choose a status code to respond back to the requester.)  

* This table show you what results may the callback receive:   

|       err      |      data        |       Description                                                  |  
|----------------|------------------|--------------------------------------------------------------------|  
| Error object   | `'_notfound_'`   | _Resource_ not found.                                              |  
| Error object   | `'_unwritable_'` | _Resource_ is unwritable.                                          |  
| Error object   | `'_exec_'`       | _Resource_ is unwritable (Because it is an executable _Resource_). |  
| `null`         | Depends          | _Resource_ is successfully write.                                  |  


**Returns:**  

* (_none_)  

**Examples:** 

```js
so.write('actuation', 0, 'onOff', 1, function (err, data) {
    if (!err)
        console.log(data);  // 1
});

so.write('temperature', 1, 'sensorValue', 26, function (err, data) {
    if (err) {
        console.log(err);   // Error: 'Resource is unwritable.'
        console.log(data);  // _unwritable_
    }
});

so.write('presence', 3, 'busyToClearDelay', function (err, data) {
    if (err) {
        console.log(err);   //  Error: 'Resource not found.'
        console.log(data);  // '_notfound_'
    }
});

so.write('barometer', 6, 'resetMinMaxMeaValues', function (err, data) {
    if (err) {
        console.log(err);   //  Error: 'Resource is unwritable.'
        console.log(data);  // '_exec_'
    }
});
```

*************************************************
<a name="API_exec"></a>
### exec(oid, iid, rid, args, callback)
Execute the specified _Resource_. The executable _Resource_ is a procedure you've defined, for example, blinking a led for _N_ times when the _Resource_ is invoked.  

**Arguments:**  

1. `oid` (_String_ | _Number_): _Object Id_ of the target.  
2. `iid` (_String_ | _Number_): _Object Instance Id_ of the target.  
3. `rid` (_String_ | _Number_): _Resource Id_ of the target.   
4. `args` (_Array_): The parameters required by the procedure.  
5. `callback` (_Function_): `function (err, data) { ... }`. Will be called when execution is performed or any error occurs, where `data` is anything your procedure like to return back. For example, when a blinking led procedure starts, you may like to return an object `{ status: 'ok', led: 6, times: 10 }` to the callback to tell something about this execution.  
  
* This table show you what results may the callback receive:   

|       err      |      data          |       Description                                                  |  
|----------------|--------------------|--------------------------------------------------------------------|  
| Error object   | `'_notfound_'`     | _Resource_ not found.                                              |  
| Error object   | `'_unexecutable_'` | _Resource_ is unexecutable.                                        |  
| Error object   | `'_badarg_'`       | Input arguments is not an array.                                   |  
| `null`         | Depends            | _Resource_ is successfully executed, `data` depends on your will.  |  

**Returns:**  

* (_none_)  

**Examples:** 

```js
// Assume we have initialized an Object Instance like this:
so.init('foo_object', 0, {
    foo: 60,
    bar: 'hello',
    blink: {
        exec: function (args, cb) {
            var ledPin = args[0],
                times = args[1];

            myHardwareController.blinkLed(ledPin, times, function (err) {
                if (err)
                    cb(err);
                else
                    cb(null, { status: 'ok', led: ledPin, times: times });
            });
        }
    }
});

// Execute the blink Resource on it
so.exec('foo_object', 0, 'blink', [ 3, 10 ], function (err, data) {
    if (!err)
        console.log(data);  // { status: 'ok', led: 3, times: 10 }
});

// Execute a Resource that doesn't exist
so.exec('foo_object', 0, 'show', [], function (err, data) {
    if (err) {
        console.log(err);   // Error: 'Resource not found.'
        console.log(data);  // '_notfound_'
    }
});
```

*************************************************
<a name="API_dump"></a>
### dump([oid[, iid][, opt],] callback)
Asynchronously dump data from `so`. This method uses the asynchronous `read()` under the hood.  

* Given with `oid`, `iid`, and a `callback` to dump data of an _Object Instance_.  
    - `dump(oid, iid, function (err, data) {})`
    - `dump(oid, iid, { restrict: true }, function (err, data) {})`
* Given with `oid` and a `callback` to dump data of an _Object_.  
    - `dump(oid, function (err, data) {})`
    - `dump(oid, { restrict: true }, function (err, data) {})`
* Given with only a `callback` to dump data of whole smart object.  
    - `dump(function (err, data) {})`
    - `dump({ restrict: true }, function (err, data) {})`

**Arguments:**  

1. `oid` (_String_ | _Number_): _Object Id_ of the target.  
2. `iid` (_String_ | _Number_): _Object Instance Id_ of the target.  
3. `callback` (_Function_): `function (err, data) { }`.  
4. `opt` (_Object_): An option used to dump _Resources_ in restrict mode. Default is `{ restrict: false }` if not given.  

**Returns:**  

* (none)

**Examples:** 

```js
// Dump Object Instance: 'temperature' sensor with iid = 18
so.dump('temperature', 18, function (err, data) {
    if (!err)
        console.log(data);
    /*
    {
        sensorValue: 301,
        units : 'K'
    }
    */
});

// Dump Object: all 'temperature' sensors
so.dump('temperature', function (err, data) {
    if (!err)
        console.log(data);
    /*
    {
        '0': {
            sensorValue: 31,
            units : 'C'
        },
        '1': {
            sensorValue: 75,
            units : 'F'
        },
        '18': {
            sensorValue: 301,
            units : 'K'
        }
    }
    */
});

// Dump whole smart object
so.dump(function (err, data) {
    if (!err)
        console.log(data);
    /*
    {
        temperature: {
            '0': {
                sensorValue: 31,
                units : 'C'
            },
            '1': {
                sensorValue: 75,
                units : 'F'
            },
            '18': {
                sensorValue: 301,
                units : 'K'
            }
        }
    }
    */
});
```

*************************************************
<a name="API_dumpSync"></a>
### dumpSync([oid[, iid]])
Synchronously dump data from `so`. This method uses the synchronous `get()` under the hood. This method **should only be used at server-side** (since at server-side, all stored _Objects_ are simply data pieces).  

* Given with both `oid` and `iid` to dump data of an _Object Instance_.  
* Given with only `oid` to dump data of an _Object_.  
* Given with no ids to dump data of whole _Smart Object_.  

**Arguments:**  

1. `oid` (_String_ | _Number_): _Object Id_ of the target.  
2. `iid` (_String_ | _Number_): _Object Instance Id_ of the target.  

**Returns:**  

* (_Object_): The dumped data, can be from an _Object Instance_, an _Object_, or whole smart object.  

**Examples:** 

* Example at client-side
```js
// Dump Object: all 'temperature' sensors
so.dumpSync('temperature');
/*
{
    '0': {
        sensorValue: 31,
        units : 'C'
    },
    '1': {
        sensorValue: {
            read: '_read_'  // a read method will be dumped to a string '_read_'
        },
        units : 'F'
    },
    '18': {
        sensorValue: 301,
        units : 'K'
    }
}
*/
```

* Examples at server-side
```js
// Assume we are at server-side.
var myDevice = myController.find('0x12AE3B4D77886644'); // find the device
var so = myDevice.getSmartObject();                     // get the smart object on the device

// Dump Object Instance: 'temperature' sensor with iid = 18
so.dumpSync('temperature', 18);
/*
{
    sensorValue: 301,
    units : 'K'
}
*/

// Dump Object: all 'temperature' sensors
so.dumpSync('temperature');
/*
{
    '0': {
        sensorValue: 31,
        units : 'C'
    },
    '1': {
        sensorValue: 75,
        units : 'F'
    },
    '18': {
        sensorValue: 301,
        units : 'K'
    }
}
*/

// Dump whole smart object
so.dumpSync();
/*
{
    temperature: {
        '0': {
            sensorValue: 31,
            units : 'C'
        },
        '1': {
            sensorValue: 75,
            units : 'F'
        },
        '18': {
            sensorValue: 301,
            units : 'K'
        }
    },
    ...
}
*/
```
*************************************************
<a name="API_isReadable"></a>
### isReadable(oid, iid, rid)
To see if a _Resource_ is readable.  

**Arguments:**  

1. `oid` (_String_ | _Number_): _Object Id_ of the target.  
2. `iid` (_String_ | _Number_): _Object Instance Id_ of the target.  
3. `rid` (_String_ | _Number_): _Resource Id_ of the target.   

**Returns:**  

* (_Boolean_): Returns `true` if the _Resource_ is readable, otherwise `false`.  

**Examples:** 

```js
so.isReadable('temperature', 8, 'sensorValue');     // true
```

*************************************************
<a name="API_isWritable"></a>
### isWritable(oid, iid, rid)
To see if a _Resource_ is writable.  

**Arguments:**  

1. `oid` (_String_ | _Number_): _Object Id_ of the target.  
2. `iid` (_String_ | _Number_): _Object Instance Id_ of the target.  
3. `rid` (_String_ | _Number_): _Resource Id_ of the target.   

**Returns:**  

* (_Boolean_): Returns `true` if the _Resource_ is writable, otherwise `false`.  

**Examples:** 

```js
so.isWritable('temperature', 8, 'sensorValue');     // false
```

*************************************************
<a name="API_isExecutable"></a>
### isExecutable(oid, iid, rid)
To see if a _Resource_ is executable.  

**Arguments:**  

1. `oid` (_String_ | _Number_): _Object Id_ of the target.  
2. `iid` (_String_ | _Number_): _Object Instance Id_ of the target.  
3. `rid` (_String_ | _Number_): _Resource Id_ of the target.   

**Returns:**  

* (_Boolean_): Returns `true` if the _Resource_ is executable, otherwise `false`.  

**Examples:**

```js
so.isExecutable('temperature', 8, 'sensorValue');   // false
```
