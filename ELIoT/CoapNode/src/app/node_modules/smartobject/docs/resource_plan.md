## Resources Planning Tutorial

This document will show you how to organize your _Resources_ and how to abstract hardware into higher-level smart objects. To initialize your _Object Instance_, the only API you'll come across is `init(oid, iid, resrcs[, setup])`, where

* `oid` is the _Object Id_
* `iid` is the _Object Instance Id_
* `resrcs` is an object to wrap your _IPSO Resources_ up. Each key in `resrcs` object is the `rid` and the value is the corresponding _Resource Value_. A protected resource `_state` is an object where you can maintain some private information or inner state inside the _Object Instance_.  
* `setup` is a function that allows you to set some inner things up for the _Object Instance_. You can use `this._state` to access the inner state, use `this.parent` to get the `so`, and use `this.parent.hal` to access your hardware.  
  
<br />

The simplest case for a _Resource Value_ is being a primitive, like a number, a string, or a bool. 
But if a _Resource_ is something that needs to be read from hardware I/O, how do we do with reading it? You can give your _Resource_ a **spec** object to tell the smart object of how to do it:  

> A **spec** object, which can have _**read**_, _**write**_, or _**exec**_ method(s) in it, is where you can inject the specific operations to tell the smart object of how to access your _Resource_.  
  
<br />

Let's figure it out step by step:  

1. [_Resource Value_ is a primitive](#r_is_primitive)
    - Let's take a temperature sensor for example
    - Polling it, but how fast?
2. [_Resource Value_ is a **spec** object](#r_is_spec)
    - Readable Resource
    - Writable Resource
    - Readable and Writable Resource
    - Executable Resource
  
Please see our [wiki](https://github.com/PeterEB/smartobject/wiki) if you like some practical examples.

<br />


<a name="r_is_primitive"></a>
### 1. _Resource Value_ is a primitive

#### 1.1 Let's take a temperature sensor for example:
```js
// oid = 'temperature', iid = 0
so.init('temperature', 0, {
    sensorValue: 20,
    units: 'cel'
});
```
  
********************************************
#### 1.2 Polling it, but how fast?
- You may think that the temperature value is time-varying, and surely just giving it a number is not a good idea. Developers are responsible for making this sensor play correctly. The simplest way is to poll the sensor and update the sensed value to the smart object regularly:
    ```js
    // Here, I use setInterval() to poll an analog input pin per 60 seconds,
    // and write the sensed value to the corresponding Resource

    setInterval(function () {
        var analogVal = analogPin0.read();
        so.write('temperature', 0, 'sensorValue', analogVal, function (err, val) {
            if (!err)
                // value written
        });
    }, 60*1000);
    ```
- The problem of polling is that the requester **may not always get the newest value** each time it requests for the 'sensorValue'. A solution is to poll the sensor more frequently, e.g., every 100ms, but I think you never want to do so to keep your device busy, and this is where the **spec** object comes in.  
  
********************************************
<br />

<a name="r_is_spec"></a>
### 2. _Resource Value_ is a **spec** object

**smartobject** allows a _Resource Value_ to be an object with `read` and/or `write` method(s). You can tell `so` how to read/write your _Resource_ through this kind of method(s). Each time someone requests for the _Resource_, `so` will invoke the read() method on that _Resource_ to get its current value, e.g. reading from a gpio immediately.  

#### 2.1 Readable Resource  
It is very simple to use this pattern. The first thing you need to know is that the signature of `read` method is `function(cb)`, where `cb(err, value)` is an err-back function that **you should call** and pass the read value through its second argument when read operation accomplishes. If any error occurs, pass the error through the first argument.  

Let's go back to the previous example and make a modification (here, I put my hardware components to `hal` in `so`):  
  
```js
var m = require('mraa');

var so = new SmartObject({
    tempSensor: new m.Aio(0)
});

so.init('temperature', 0, {
    sensorValue: {
        read: function (cb) {
            var hal = this.parent.hal;
            var analogValue = hal.tempSensor.read();
            // Maybe some calculation is needed to get the right temperature...

            cb(null, analogValue)
        }
    },
    units: 'cel'
});
```
  
See, it's simple. If you define this object with a read method, this _Resource_ will be inherently readable.  
  
********************************************
#### 2.2 Writable Resource  
The pattern for a writable _Resource_ is similar. The signature of `write` method is `function(value, cb)`, where `value` is the value to write to this _Resource_ and `cb(err, value)` is an err-back function that you should call and pass the written value through its second argument. Example again:  
  
```js
var m = require('mraa');

var so = new SmartObject({
    tempSensor: new m.Aio(0),
    actuator: new m.Gpio(5)
}, function () {
    this.hal.actuator.dir(m.DIR_OUT);   // setup for gpio direction
});

so.init('actuation', 6, {
    onOff: {
        write: function (value, cb) {
            var hal = this.parent.hal;

            value = value ? 1 : 0;
            hal.actuator.write(value);

            cb(null, value);
        }
    }
});
```
  
In this example, we only define the write method for the _Resource_, thus it is writable but not readable. If someone is trying to read this _Resource_, `so` will give him/her an error and a special value of string `'_unreadable_'` passing to the second argument of `callback`.  
  
********************************************
#### 2.3 Readable and writable Resource  

If this _Resource_ is both readable and writable, you should give both of read and write methods to it:
  
```js
var b = require('bonescript');

var so = new SmartObject(function () {
    this.hal.actuatorPin = 'P8_13'; // this.hal is an empty object by default
    b.pinMode(this.hal.actuatorPin, b.OUTPUT);
});

so.init('actuation', 6, {
    onOff: {
        read: function (cb) {
            var actuatorPin = this.parent.hal.actuatorPin;

            b.digitalRead(actuatorPin, function (result) {
                if (result.err)
                    cb(err);
                else
                    cb(null, result.value);
            });
        },
        write: function (value, cb) {
            var actuatorPin = this.parent.hal.actuatorPin;
            value = value ? b.HIGH : b.LOW;

            b.digitalWrite(actuatorPin, value, function (result) {
                if (result.err)
                    cb(err);
                else
                    cb(null, value);
            });
        }
    }
});
```
  
Ok, good! You've not only learned how to read/write a _Resource_ but also learned how to do the **Access Control** on a _Resource_. If the _Resource Value_ is a primitive, **smartobject** will follow the access rules from IPSO specification. If your _Resource Value_ is a primitive and you don't want to follow the default access rules, you can wrap it up with this kind of special object we've just introduced. See this example:
  
```js
var tempVal = 26;

so.init('temperature', 0, {
    sensorValue: {
        read: function (cb) {
            cb(null, tempVal);
        }
    },
    units: 'cel'
});
```
  
You can also take the `tempVal` as the inner state of the _Object Instance_:
  
```js
so.init('temperature', 0, {
    _state: {
        tempVal: 26
    },
    sensorValue: {
        read: function (cb) {
            cb(null, this._state.tempVal);
        },
        write: function (val, cb) {
            this._state.tempVal = val;
            cb(null, this._state.tempVal);
        }
    },
    units: 'cel'
});
```

Next, let's take a look at something really cool - an _executable Resource_.  

  
********************************************
#### 2.4 _Executable Resource_

This kind of _Resource_ allows you to issue a procedure on the `so`, for example, ask your device to blink a LED for 10 times. You can define some useful and interesting remote procedure calls (RPCs) with executable _Resources_.  

To do so, give your _Resource_ an object with the `exec` method. In this case, the _Resource_ will be inherently an executable one, you will get an error and a special value of string `'_exec_'` when reading from or writing to it. This means that read and write methods are ineffective to an _executable Resource_ even if you do give an object with these two methods to the _Resource_.  

If the _Resource_ is not an executable one, **smartoject** will respond a error and a special value of `'_unexecutable_'` passing to the second argument of `callback` when you trying to invoke it.  
  
<br />
#### Example: An executable _Resource_ to blink a led  

It's time to show you an example. Assume that we have an _executable Resource_ `function(t)` on the device to start blinking the led with `t` times.  
  
```js
var m = require('mraa');

var so = new SmartObject({
    led: new m.Gpio(2),
    blinkLed: null  // we'll define this drive in the setup function
}, function () {
    this.hal.led.dir(m.DIR_OUT);   // setup for gpio direction

    this.hal.blinkLed = function (t) {
        var led = this.hal.led;
        // logic of blinking an led
    };
});

so.init('myObject', 0, {
    blink: {
        exec: function (t, cb) {
            var blinkLed = this.hal.blinkLed;
            blinkLed(t);            // invoke the procedure
            cb(null, 'blinking');   // cb(err, data) where data is something you'd like to respond back  
        }
    }
});
```
  
The signature of `exec` method is `function(...[, cb])`, and
* The number of arguments depends on your own definition  
* The callback `cb(err, data)` should be called when your procedure is correctly invoked  

  
<br />
#### Executable Resource is Cool

An _Executable Resource_ is a necessary if you like to do something complicated.  

Think of that how do you blink a certain led with arbitrary times if you are just using general readable/writable _Resources_? That can be a pain. IoT is not just about reading something from or writing something to machines. An Executable Resource is very powerful and it lets your machines do more things and be more automatic.  
