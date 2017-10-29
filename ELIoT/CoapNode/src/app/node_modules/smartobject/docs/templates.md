## Code Templates

This document provides many templates of IPSO-defined devices [(Smart Objects starter pack 1.0)](http://www.ipso-alliance.org/smart-object-guidelines/) for your reference in defining gadgets with **smartobject**.  
* Each template gives a code snippet of how to initialize an _Object Instance_ with its `oid` and `iid`, and lists every _Resource_ the _Object Instance_ **may** have.  
* In a code snippet, commented lines are optional _Resources_. You are free to uncomment and define those optional _Resources_ you like to use within an _Object Instance_.  
* A phrase `< rid number, access, data type { range or enum }, unit >` tells the numeric id, access permission, and data type of a _Resource_ defined by IPSO.  
* Just copy and paste the snippets, then make some changes to fit your needs.  

### Catalog of _Objects_


|                                            |                                              |                                               |                                                   |
|--------------------------------------------|----------------------------------------------|-----------------------------------------------|---------------------------------------------------|
| 3200 [Digital Input](#tmpl_digitalInput)   | 3201 [Digital Output](#tmpl_digitalOutput)   | 3202 [Analog Input](#tmpl_analogInput)        | 3203 [Analog Output](#tmpl_analogOutput)          |
| 3300 [Generic Sensor](#tmpl_genericSensor) | 3301 [Illuminance Sensor](#tmpl_illumSensor) | 3302 [Presence Sensor](#tmpl_presenceSensor)  | 3303 [Temperature Sensor](#tmpl_temperature)      |
| 3304 [Humidity Sensor](#tmpl_humidity)     | 3305 [Power Measurement](#tmpl_pwrMea)       | 3306 [Actuation](#tmpl_actuation)             | 3308 [Set Point](#tmpl_setPoint)                  |
| 3310 [Load Control](#tmpl_loadCtrl)        | 3311 [Light Control](#tmpl_lightCtrl)        | 3312 [Power Control](#tmpl_pwrCtrl)           | 3313 [Accelerometer](#tmpl_accelerometer)         |
| 3314 [Magnetometer](#tmpl_magnetometer)    | 3315 [Barometer](#tmpl_barometer)            | 3316 [Voltage](#tmpl_common)                  | 3317 [Current](#tmpl_common)                      |
| 3318 [Frequency](#tmpl_common)             | 3319 [Depth](#tmpl_common)                   | 3320 [Percentage](#tmpl_common)               | 3321 [Altitude](#tmpl_common)                     |
| 3322 [Load](#tmpl_common)                  | 3323 [Pressure](#tmpl_common)                | 3324 [Loudness](#tmpl_common)                 | 3325 [Concentration](#tmpl_common)                |
| 3326 [Acidity](#tmpl_common)               | 3327 [Conductivity](#tmpl_common)            | 3328 [Power](#tmpl_common)                    | 3329 [Power Factor](#tmpl_common)                 |
| 3330 [Distance](#tmpl_common)              | 3331 [Energy](#tmpl_energy)                  | 3332 [Direction](#tmpl_direction)             | 3333 [Time](#tmpl_time)                           |
| 3334 [Gyrometer](#tmpl_gyrometer)          | 3335 [Colour](#tmpl_colour)                  | 3336 [GPS Location](#tmpl_gpsLocation)        | 3337 [Positioner](#tmpl_positioner)               |
| 3338 [Buzzer](#tmpl_buzzer)                | 3339 [Audio Clip](#tmpl_clip)                | 3340 [Timer](#tmpl_timer)                     | 3341 [Addressable Text Display](#tmpl_display)    |
| 3342 [On/Off Switch](#tmpl_switch)         | 3343 [Level Controller](#tmpl_level)         | 3344 [Up/Down Controller](#tmpl_updown)       | 3345 [Multiple Axis Joystick](#tmpl_joystick)     |
| 3346 [Rate](#tmpl_rate)                    | 3347 [Push Button](#tmpl_button)             | 3348 [Multi-state Selector](#tmpl_mselector)  |                                                   |
|                                            |                                              |                                               |                                                   |


********************************************
<a name="tmpl_digitalInput"></a>
### 3200 Digital Input
  
```js
// Digital Input (oid = 3200 or 'dIn')
so.init('dIn', 0, {
    dInState: {                     // < rid = 5500, R, Boolean >
        read: function (cb) {}
    },
    // counter: ,                   // < rid = 5501,  R, Integer >
    // dInPolarity: ,               // < rid = 5502, RW, Boolean >
    // debouncePeriod: ,            // < rid = 5503, RW, Integer, ms >
    // edgeSelection: ,             // < rid = 5504, RW, Integer { 1: fall, 2: rise, 3: both } >
    // counterReset: ,              // < rid = 5505,  E, Opaque >
    // appType: ,                   // < rid = 5750, RW, String >
    // sensorType:                  // < rid = 5751,  R, String >
});
```
  
********************************************
<a name="tmpl_digitalOutput"></a>
### 3201 Digital Output
  
```js
// Digital Output (oid = 3201 or 'dOut')
so.init('dOut', 0, {
    dOutState: {                    // < rid = 5550, RW, Boolean >
        read: function (cb) {},
        write: function (value, cb) {}
    },
    // dOutpolarity: ,              // < rid = 5551, RW, Boolean { 0: normal, 1: reversed } >
    // appType:                     // < rid = 5750, RW, String >
});
```
  
********************************************
<a name="tmpl_analogInput"></a>
### 3202 Analog Input
  
```js
// Analog Input (oid = 3202 or 'aIn')
so.init('aIn', 0, {
    aInCurrValue: {                 // < rid = 5600, R, Float >
        read: function (cb) {}
    },
    // minMeaValue: ,               // < rid = 5601,  R, Float >
    // maxMeaValue: ,               // < rid = 5602,  R, Float >
    // minRangeValue: ,             // < rid = 5603,  R, Float >
    // maxRangeValue: ,             // < rid = 5604,  R, Float >
    // resetMinMaxMeaValues: ,      // < rid = 5605,  E, Opaque >
    // appType: ,                   // < rid = 5750, RW, String >
    // sensorType:                  // < rid = 5751,  R, String >
});
```
  
********************************************
<a name="tmpl_analogOutput"></a>
### 3203 Analog Output
  
```js
// Analog Output (oid = 3203 or 'aOut')
so.init('aOut', 0, {
    aOutCurrValue: {                // < rid = 5650, RW, Float >
        read: function (cb) {},
        write: function (value, cb) {}
    },
    // minRangeValue: ,             // < rid = 5603,  R, Float >
    // maxRangeValue: ,             // < rid = 5604,  R, Float >
    // appType:                     // < rid = 5750, RW, String >
});
```
  
********************************************
<a name="tmpl_genericSensor"></a>
### 3300 Generic Sensor
  
```js
// Generic Sensor (oid = 3300 or 'generic')
so.init('generic', 0, {
    sensorValue: {                  // < rid = 5700, R, Float >
        read: function (cb) {}
    },
    // units: ,                     // < rid = 5701,  R, String >
    // minMeaValue: ,               // < rid = 5601,  R, Float >
    // maxMeaValue: ,               // < rid = 5602,  R, Float >
    // minRangeValue: ,             // < rid = 5603,  R, Float >
    // maxRangeValue: ,             // < rid = 5604,  R, Float >
    // resetMinMaxMeaValues: ,      // < rid = 5605,  E, Opaque >
    // appType: ,                   // < rid = 5750, RW, String >
    // sensorType:                  // < rid = 5751,  R, String >
});
```
  
********************************************
<a name="tmpl_illumSensor"></a>
### 3301 Illuminance Sensor
  
```js
// Illuminance Sensor (oid = 3301 or 'illuminance')
so.init('illuminance', 0, {
    sensorValue: {                  // < rid = 5700, R, Float >
        read: function (cb) {}
    },
    // units: ,                     // < rid = 5701, R, String >
    // minMeaValue: ,               // < rid = 5601, R, Float >
    // maxMeaValue: ,               // < rid = 5602, R, Float >
    // minRangeValue: ,             // < rid = 5603, R, Float >
    // maxRangeValue: ,             // < rid = 5604, R, Float >
    // resetMinMaxMeaValues:        // < rid = 5605, E, Opaque >
});
```
  
********************************************
<a name="tmpl_presenceSensor"></a>
### 3302 Presence Sensor
  
```js
// Presence Sensor (oid = 3302 or 'presence')
so.init('presence', 0, {
    dInState: {                     // < rid = 5500, R, Boolean >
        read: function (cb) {}
    },
    // counter: ,                   // < rid = 5501,  R, Integer >
    // counterReset: ,              // < rid = 5505,  E, Opaque >
    // sensorType: ,                // < rid = 5751,  R, String >
    // busyToClearDelay: ,          // < rid = 5903, RW, Integer, ms >
    // clearToBusyDelay:            // < rid = 5904  RW, Integer, ms >
});
```
  
********************************************
<a name="tmpl_temperature"></a>
### 3303 Temperature Sensor
  
```js
// Temperature Sensor (oid = 3303 or 'temperature')
so.init('temperature', 0, {
    sensorValue: {                  // < rid = 5700, R, Float >
        read: function (cb) {}
    },
    // units: ,                     // < rid = 5701, R, String >
    // minMeaValue: ,               // < rid = 5601, R, Float >
    // maxMeaValue: ,               // < rid = 5602, R, Float >
    // minRangeValue: ,             // < rid = 5603, R, Float >
    // maxRangeValue: ,             // < rid = 5604, R, Float >
    // resetMinMaxMeaValues:        // < rid = 5605, E, Opaque >
});
```
  
********************************************
<a name="tmpl_humidity"></a>
### 3304 Humidity Sensor
  
```js
// Humidity Sensor (oid = 3304 or 'humidity')
so.init('humidity', 0, {
    sensorValue: {                  // < rid = 5700, R, Float >
        read: function (cb) {}
    },
    // units: ,                     // < rid = 5701, R, String >
    // minMeaValue: ,               // < rid = 5601, R, Float >
    // maxMeaValue: ,               // < rid = 5602, R, Float >
    // minRangeValue: ,             // < rid = 5603, R, Float >
    // maxRangeValue: ,             // < rid = 5604, R, Float >
    // resetMinMaxMeaValues:        // < rid = 5605, E, Opaque >
});
```
  
********************************************
<a name="tmpl_pwrMea"></a>
### 3305 Power Measurement
  
```js
// Power Measurement (oid = 3305 or 'pwrMea')
so.init('pwrMea', 0, {
    instActivePwr: {                // < rid = 5800, R, Float, Wh >
        read: function (cb) {}
    },
    // minMeaActivePwr: ,           // < rid = 5801,  R, Float, W >
    // maxMeaActivePwr: ,           // < rid = 5802,  R, Float, W >
    // minRangeActivePwr: ,         // < rid = 5803,  R, Float, W >
    // maxRangeActivePwr: ,         // < rid = 5804,  R, Float, W >
    // cumulActivePwr: ,            // < rid = 5805,  R, Float, Wh >
    // activePwrCal: ,              // < rid = 5806,  W, Float, W >
    // instReactivePwr: ,           // < rid = 5810,  R, Float, VAR >
    // minMeaReactivePwr: ,         // < rid = 5811,  R, Float, VAR >
    // maxMeaReactivePwr: ,         // < rid = 5812,  R, Float, VAR >
    // minRangeReactivePwr: ,       // < rid = 5813,  R, Float, VAR >
    // maxRangeReactivePwr: ,       // < rid = 5814,  R, Float, VAR >
    // resetMinMaxMeaValues: ,      // < rid = 5605,  E, Opaque >
    // cumulReactivePwr: ,          // < rid = 5815,  R, Float, VARh >
    // reactivePwrCal: ,            // < rid = 5816,  W, Float, VAR >
    // pwrFactor: ,                 // < rid = 5820,  R, Float >
    // currCal: ,                   // < rid = 5821, RW, Float >
    // resetCumulEnergy: ,          // < rid = 5822,  E, Opaque >
});
```
  
********************************************
<a name="tmpl_actuation"></a>
### 3306 Actuation
  
```js
// Actuation (oid = 3306 or 'actuation')
so.init('actuation', 0, {
    onOff: {                        // < rid = 5850, RW, Boolean { 0: off, 1: on } >
        read: function (cb) {},
        write: function (value, cb) {}
    },
    // dimmer: ,                    // < rid = 5851, RW, Integer { 0 ~ 100 }, % >
    // onTime: ,                    // < rid = 5852, RW, Integer, s >
    // mstateOut: ,                 // < rid = 5853, RW, String >
    // appType:                     // < rid = 5750, RW, String >
});
```
  
********************************************
<a name="tmpl_setPoint"></a>
### 3308 Set Point
  
```js
// Set Point (oid = 3308 or 'setPoint')
so.init('setPoint', 0, {
    setPointValue: {                // < rid = 5900, RW, Float >
        read: function (cb) {},
        write: function (value, cb) {}
    },
    // colour: ,                    // < rid = 5706, RW, String >
    // units: ,                     // < rid = 5701,  R, String >
    // appType:                     // < rid = 5750, RW, String >
});
```
  
********************************************
<a name="tmpl_loadCtrl"></a>
### 3310 Load Control
  
```js
// Load Control (oid = 3310 or 'loadCtrl')
so.init('loadCtrl', 0, {
    eventId: {                      // < rid = 5823, RW, String >
        read: function (cb) {},
        write: function (value, cb) {}
    },
    startTime: {                    // < rid = 5824, RW, Time >
        read: function (cb) {},
        write: function (value, cb) {}
    },
    durationInMin: {                // < rid = 5825, RW, Integer, min >
        read: function (cb) {},
        write: function (value, cb) {}
    },
    // criticalLevel: ,             // < rid = 5826, RW, Integer { 0: normal, 1: warning, 2: danger, 3: fatal } >
    // avgLoadAdjPct: ,             // < rid = 5827, RW, Integer { 0 ~ 100 }, % >
    // dutyCycle:                   // < rid = 5828, RW, Interger { 0 ~ 100 }, % >
});
```
  
********************************************
<a name="tmpl_lightCtrl"></a>
### 3311 Light Control
  
```js
// Light Control (oid = 3311 or 'lightCtrl')
so.init('lightCtrl', 0, {
    onOff: {                        // < rid = 5850, RW, Boolean { 0: off, 1: on } >
        read: function (cb) {},
        write: function (value, cb) {}
    },
    // dimmer: ,                    // < rid = 5851, RW, Integer { 0 ~ 100 }, %  >
    // colour: ,                    // < rid = 5706, RW, String >
    // units: ,                     // < rid = 5701,  R, String >
    // onTime: ,                    // < rid = 5852, RW, Integer, s >
    // cumulActivePwr: ,            // < rid = 5805,  R, Float, Wh >
    // pwrFactor:                   // < rid = 5820,  R, Float >
});
```
  
********************************************
<a name="tmpl_pwrCtrl"></a>
### 3312 Power Control
  
```js
// Power Control (oid = 3312 or 'pwrCtrl')
so.init('pwrCtrl', 0, {
    onOff: {                        // < rid = 5850, RW, Boolean { 0: off, 1: on } >
        read: function (cb) {},
        write: function (value, cb) {}
    },
    // dimmer: ,                    // < rid = 5851, RW, Integer { 0 ~ 100 }, % >
    // onTime: ,                    // < rid = 5852, RW, Integer, s >
    // cumulActivePwr: ,            // < rid = 5805,  R, Float, Wh >
    // pwrFactor:                   // < rid = 5820,  R, Float >
});
```
  
********************************************
<a name="tmpl_accelerometer"></a>
### 3313 Accelerometer

```js
// Accelerometer (oid = 3313 or 'accelerometer')
so.init('accelerometer', 0, {
    xValue: {                       // < rid = 5702, R, Float >
        read: function (cb) {}
    },
    // yValue: ,                    // < rid = 5703, R, Float >
    // zValue: ,                    // < rid = 5704, R, Float >
    // units: ,                     // < rid = 5701, R, String >
    // minRangeValue: ,             // < rid = 5603, R, Float >
    // maxRangeValue:               // < rid = 5604, R, Float >
});
```
  
********************************************
<a name="tmpl_magnetometer"></a>
### 3314 Magnetometer
  
```js
// Magnetometer (oid = 3314 or 'magnetometer')
so.init('magnetometer', 0, {
    xValue: {                       // < rid = 5702, R, Float >
        read: function (cb) {}
    },
    // yValue: ,                    // < rid = 5703, R, Float >
    // zValue: ,                    // < rid = 5704, R, Float >
    // units:,                      // < rid = 5701, R, String >
    // compassDir:                  // < rid = 5705, R, Float { 0 ~ 360 }, deg >
});
```
  
********************************************
<a name="tmpl_barometer"></a>
### 3315 Barometer
  
```js
// Barometer (oid = 3315 or 'barometer')
so.init('barometer', 0, {
    sensorValue: {                  // < rid = 5700, R, Float >
        read: function (cb) {}
    },
    // units: ,                     // < rid = 5701, R, String >
    // minMeaValue: ,               // < rid = 5601, R, Float >
    // maxMeaValue: ,               // < rid = 5602, R, Float >
    // minRangeValue: ,             // < rid = 5603, R, Float >
    // maxRangeValue: ,             // < rid = 5604, R, Float >
    // resetMinMaxMeaValues:        // < rid = 5605, E, Opaque >
});
```

********************************************
<a name="tmpl_common"></a>
### 3316 Voltage, 3317 Current, 3318 Frequency, 3319 Depth, 3320 Percentage, 3321 Altitude, 3322 Load, 3323 Pressure, 3324 Loudness, 3325 Concentration, 3326 Acidity, 3327 Conductivity, 3328 Power, 3329 Power Factor, 3330 Distance
  
```js
// 'xxx' Can be 'voltage', 'current', 'frequency', ..., and 'distance'
so.init('xxx', 0, {
    sensorValue: {                  // < rid = 5700, R, Float >
        read: function (cb) {}
    },
    // units: ,                     // < rid = 5701,  R, String >
    // minMeaValue: ,               // < rid = 5601,  R, Float >
    // maxMeaValue: ,               // < rid = 5602,  R, Float >
    // minRangeValue: ,             // < rid = 5603,  R, Float >
    // maxRangeValue: ,             // < rid = 5604,  R, Float >
    // resetMinMaxMeaValues: ,      // < rid = 5605,  E, Opaque >
    // calOffset: ,                 // < rid = 5535, RW, Float >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_energy"></a>
### 3331 Energy
  
```js
// Energy (oid = 3331 or 'energy')
so.init('energy', 0, {
    cumulActivePwr: {               // < rid = 5700,  R, Float >
        read: function (cb) {}
    }, 
    // units: ,                     // < rid = 5701,  R, String >
    // resetCumulEnergy: ,          // < rid = 5605,  E, Opaque >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_direction"></a>
### 3332 Direction
  
```js
// Direction (oid = 3332 or 'direction')
so.init('direction', 0, {
    compassDir: {                   // < rid = 5705,  R, Float >
        read: function (cb) {}
    },
    // compassDir: 5705,
    // minMeaValue: ,               // < rid = 5601,  R, Float >
    // maxMeaValue: ,               // < rid = 5602,  R, Float >
    // resetMinMaxMeaValues: ,      // < rid = 5605,  E, Opaque >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_time"></a>
### 3333 Time
  
```js
// Time (oid = 3333 or 'time')
so.init('time', 0, {
    currentTime: {                  // < rid = 5506, RW, Time >
        read: function (cb) {},
        write: function (value, cb) {}
    },
    // fracTime:                    // < rid = 5507, RW, Float {1} >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_gyrometer"></a>
### 3334 Gyrometer
  
```js
// Gyrometer (oid = 3334 or 'gyrometer')
so.init('gyrometer', 0, {
    xValue: {                       // < rid = 5702,  R, Float >
        read: function (cb) {}
    },
    // yValue: ,                    // < rid = 5703,  R, Float >
    // zValue: ,                    // < rid = 5704,  R, Float >
    // units: ,                     // < rid = 5701,  R, String >
    // minRangeValue: ,             // < rid = 5603,  R, Float >
    // maxRangeValue: ,             // < rid = 5604,  R, Float >
 
    // minXValue: ,                 // < rid = 5508,  R, Float >
    // maxXValue: ,                 // < rid = 5609,  R, Float >
    // minYValue: ,                 // < rid = 5510,  R, Float >
    // maxYValue: ,                 // < rid = 5611,  R, Float >
    // minZValue: ,                 // < rid = 5512,  R, Float >
    // maxZValue: ,                 // < rid = 5613,  R, Float >
    // resetMinMaxMeaValues: ,      // < rid = 5605,  E, Opaque >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_colour"></a>
### 3335 Colour
  
```js
// Colour (oid = 3335 or 'colour')
so.init('colour', 0, {
    colour: {                       // < rid = 5706, RW, String >
        read: function (cb) {},
        write: function (value, cb) {}
    },
    // units: ,                     // < rid = 5701,  R, String >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_gpsLocation"></a>
### 3336 GPS Location
  
```js
// GPS Location (oid = 3336 or 'gpsLocation')
so.init('gpsLocation', 0, {
    latitude: {                     // < rid = 5514,  R, String >
        read: function (cb) {}
    },
    longitude: {                    // < rid = 5515,  R, String >
        read: function (cb) {}
    },
    // uncertainty: ,               // < rid = 5516,  R, String >
    // compassDir: ,                // < rid = 5705,  R, Float >
    // velocity: ,                  // < rid = 5517,  R, Opaque >
    // timestamp: ,                 // < rid = 5518,  R, Time >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_positioner"></a>
### 3337 Positioner

```js
// Positioner (oid = 3337 or 'positioner')
so.init('positioner', 0, {
    currentPos: {                   // < rid = 5536, RW, Float {100} >
        read: function (cb) {},
        write: function (value, cb) {}
    },
    // transTime: ,                // < rid = 5537, RW, Float >
    // remainTime: ,               // < rid = 5538,  R, Float >
    // minMeaValue: ,              // < rid = 5601,  R, Float >
    // maxMeaValue: ,              // < rid = 5602,  R, Float >
    // resetMinMaxMeaValues:       // < rid = 5605,  E, Opaque >
    // minLimit: ,                 // < rid = 5519,  R, Float >
    // maxLimit: ,                 // < rid = 5520,  R, Float >
    // appType:                    // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_buzzer"></a>
### 3338 Buzzer

```js
// Buzzer (oid = 3338 or 'buzzer')
so.init('buzzer', 0, {
    onOff: {                        // < rid = 5850, RW, Boolean >
        read: function (cb) {},
        write: function (value, cb) {}
    },
    // level: ,                     // < rid = 5548, RW, Float {100} >
    // timeDuration: ,              // < rid = 5521, RW, Float >
    // minOffTime: ,                // < rid = 5525, RW, Float >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_clip"></a>
### 3339 Audio Clip

```js
// Audio Clip (oid = 3339 or 'audioClip')
so.init('audioClip', 0, {
    clip: {                         // < rid = 5522, RW, Opaque >
        read: function (cb) {},
        write: function (value, cb) {}
    },
    // trigger:                     // < rid = 5523,  E, Opaque >
    // level: ,                     // < rid = 5548, RW, Float {100} >
    // soundDuration: ,             // < rid = 5524, RW, Float >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_timer"></a>
### 3340 Timer

```js
// Timer (oid = 3340 or 'timer')
so.init('timer', 0, {
    timeDuration: {                 // < rid = 5521, R, Float >
        read: function (cb) {}
        write: function (value, cb) {}
    },
    // remainTime: ,                // < rid = 5538,  R, Float >
    // minOffTime: ,                // < rid = 5525, RW, Float >
    // trigger:                     // < rid = 5523,  E, Opaque >
    // onOff: ,                     // < rid = 5850, RW, Boolean >
    // counter: ,                   // < rid = 5501,  R, Integer >
    // cumulTime: ,                 // < rid = 5544, RW, Float >
    // digitalState: ,              // < rid = 5543,  R, Boolean >
    // eventCounter: ,              // < rid = 5534, RW, Integer >
    // mode: ,                      // < rid = 5526, RW, Integer {4} >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_display"></a>
### 3341 Addressable Text Display

```js
// Addressable Text Display (oid = 3341 or 'addressableTextDisplay')
so.init('addressableTextDisplay', 0, {
    text: {                         // < rid = 5527, RW, String >
        read: function (cb) {}
    },
    // xCoord:,                     // < rid = 5528, RW, Integer >
    // yCoord:,                     // < rid = 5529, RW, Integer >
    // maxXCoord:,                  // < rid = 5545,  R, Integer >
    // maxYCoord:,                  // < rid = 5546,  R, Integer >
    // clearDisplay:                // < rid = 5530,  E, Opaque >
    // contrast: ,                  // < rid = 5531, RW, Float {100} >
    // level: ,                     // < rid = 5548, RW, Float {100} >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_switch"></a>
### 3342 On/Off Switch

```js
// On/Off Switch (oid = 3342 or 'onOffSwitch')
so.init('onOffSwitch', 0, {
    dInState: {                     // < rid = 5500, R, Boolean >
        read: function (cb) {}
    },
    // counter: ,                   // < rid = 5501,  R, Integer >
    // onTime: ,                    // < rid = 5852, RW, Integer, s >
    // offTime: ,                   // < rid = 5854, RW, Integer, s >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_level"></a>
### 3343 Level Controller

```js
// Level Controller (oid = 3343 or 'levelControl')
so.init('levelControl', 0, {
    level: {                        // < rid = 5548, RW, Float {100} >
        read: function (cb) {},
        write: function (value, cb) {}
    },
    // onTime: ,                    // < rid = 5852, RW, Integer, s >
    // offTime: ,                   // < rid = 5854, RW, Integer, s >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_updown"></a>
### 3344 Up/Down Controller

```js
// Up/Down Controller (oid = 3343 or 'upDownControl')
so.init('upDownControl', 0, {
    incInputState: {                // < rid = 5532, R, Boolean >
        read: function (cb) {}
    },
    decInputState: {                // < rid = 5533, R, Boolean >
        read: function (cb) {}
    },
    // upCounter: ,                 // < rid = 5541, RW, Integer >
    // downCounter: ,               // < rid = 5542, RW, Integer >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_joystick"></a>
### 3345 Multiple Axis Joystick

```js
// Multiple Axis Joystick (oid = 3345 or 'multipleAxisJoystick')
so.init('multipleAxisJoystick', 0, {
    dInState: {                     // < rid = 5500, R, Boolean >
        read: function (cb) {}
    },
    // counter: ,                   // < rid = 5501,  R, Integer >
    // xValue: ,                    // < rid = 5702,  R, Float >
    // yValue: ,                    // < rid = 5703,  R, Float >
    // zValue: ,                    // < rid = 5704,  R, Float >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_rate"></a>
### 3346 Rate

```js
// Rate (oid = 3346 or 'rate')
so.init('rate', 0, {
    sensorValue: {                  // < rid = 5700, R, Float >
        read: function (cb) {}
    },
    // units: ,                     // < rid = 5701, R, String >
    // minMeaValue: ,               // < rid = 5601, R, Float >
    // maxMeaValue: ,               // < rid = 5602, R, Float >
    // minRangeValue: ,             // < rid = 5603, R, Float >
    // maxRangeValue: ,             // < rid = 5604, R, Float >
    // resetMinMaxMeaValues:        // < rid = 5605, E, Opaque >
    // calOffset: ,                 // < rid = 5535, RW, Float >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_button"></a>
### 3347 Push Button

```js
// Push Button (oid = 3347 or 'pushButton')
so.init('pushButton', 0, {
    dInState: {                     // < rid = 5500, R, Boolean >
        read: function (cb) {}
    },
    // counter: ,                   // < rid = 5501,  R, Integer >
    // appType:                     // < rid = 5750, RW, String >
});
```

********************************************
<a name="tmpl_mselector"></a>
### 3348 Multi-state Selector

```js
// Multi-state Selector (oid = 3348 or 'multistateSelector')
so.init('multistateSelector', 0, {
    mStateIn: {                     // < rid = 5547, R, Integer >
        read: function (cb) {}
    },
    // appType:                     // < rid = 5750, RW, String >
});
```
