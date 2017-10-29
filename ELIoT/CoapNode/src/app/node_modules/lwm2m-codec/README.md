# lwm2m-codec
A codec for lightweight M2M (LWM2M) data formats.

<br />

## Documentation  

Please visit the [Wiki](https://github.com/PeterEB/lwm2m-codec/wiki).

<br />

## Overview



<br />

## Installation

> $ npm install lwm2m-codec --save  

<br />

## Usage

TLV:

```js
var lwCodec = require(lwm2m-codec);

lwCodec.encode('tlv', '/3303/0', { 5700: 31, 5701: 'c' });
lwCodec.encode('tlv', '/temperature/0', { 5700: 31, 5701: 'c' });
lwCodec.encode('tlv', '/temperature/0', { sensorValue: 31, units: 'c' });
// Buffer <0x08, 0x00, 0x0b, 0xe4, 0x16, 0x44, 0x41, 0xf8, 0x00, 0x00, 0xe1, 0x16, 0x45, 0x63>;

lwCodec.decode('tlv', '/3303/0', new Buffer ([0x08, 0x00, 0x0b, 0xe4, 0x16, 0x44, 0x41, 0xf8, 0x00, 0x00, 0xe1, 0x16, 0x45, 0x63]));
// { sensorValue: 31, units: 'c' }
```

JSON:

```js
var lwCodec = require(lwm2m-codec);

lwCodec.encode('json', '/3303/0', { 5700: 31, 5701: 'c' });
lwCodec.encode('json', '/temperature/0', { 5700: 31, 5701: 'c' });
lwCodec.encode('json', '/temperature/0', { sensorValue: 31, units: 'c' });
// Buffer <7b 22 62 6e 22 3a 22 2f 33 33 30 33 2f 30 22 2c 22 65 22 3a 5b 7b 22 6e 22 3a 22 35 37 30 30 22 2c 22 76 22 3a 33 31 7d 2c 7b 22 6e 22 3a 22 35 37 30 31 22 2c 22 73 76 22 3a 22 63 22 7d 5d 7d 0d>;
// String {"bn":"/3303/0","e":[{"n":"5700","v":31},{"n":"5701","sv":"c"}]} 

lwCodec.decode('json', '/3303/0', new Buffer ([0x7b ,0x22 ,0x62 ,0x6e ,0x22 ,0x3a ,0x22 ,0x2f ,0x33 ,0x33 ,0x30 ,0x33 ,0x2f ,0x30 ,0x22 ,0x2c ,0x22 ,0x65 ,0x22 ,0x3a ,0x5b ,0x7b ,0x22 ,0x6e ,0x22 ,0x3a ,0x22 ,0x35 ,0x37 ,0x30 ,0x30 ,0x22 ,0x2c ,0x22 ,0x76 ,0x22 ,0x3a ,0x33 ,0x31 ,0x7d ,0x2c ,0x7b ,0x22 ,0x6e ,0x22 ,0x3a ,0x22 ,0x35 ,0x37 ,0x30 ,0x31 ,0x22 ,0x2c ,0x22 ,0x73 ,0x76 ,0x22 ,0x3a ,0x22 ,0x63 ,0x22 ,0x7d ,0x5d ,0x7d 
]));
// { sensorValue: 31, units: 'c' }
```

<br />

## License

Licensed under [MIT](https://github.com/PeterEB/lwm2m-codec/blob/master/LICENSE).
