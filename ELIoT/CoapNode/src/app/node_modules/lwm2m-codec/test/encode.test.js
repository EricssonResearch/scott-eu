var expect = require('chai').expect,
    encode = require('../lib/encode.js'),
    decode = require('../lib/decode.js'),
    cutils = require('../lib/cutils.js');
    
describe('encode - Functional Check', function () {
    it('#.encode(link)', function () {
        expect(encode('link', 'x', { 1: {  0: 'x', 1: 5 }, 2: {  0: true, 1: 0 }}, { pmin: 10, pmax: 60 })).to.be.eql('</x>;pmin=10;pmax=60,</x/1/0>,</x/1/1>,</x/2/0>,</x/2/1>');
        expect(encode('link', 'x/y', {  0: 'x', 1: 5 }, { pmin: 10, pmax: 60 })).to.be.eql('</x/y>;pmin=10;pmax=60,</x/y/0>,</x/y/1>');
        expect(encode('link', 'x/y/z', 10, { pmin: 10, pmax: 60 })).to.be.eql('</x/y/z>;pmin=10;pmax=60');
    });

    it('#.encode(tlv)', function () {
        expect(encode('tlv', '/3303', {0: {5700: 31, 5701: 'c'}, 1:{5700: 89, 5701: 'f'}})).to.be.eql(cutils.bufferFrom([0x08, 0x00, 0x0b, 0xe4, 0x16, 0x44, 0x41, 0xf8, 0x00, 0x00, 0xe1, 0x16, 0x45, 0x63, 0x08, 0x01, 0x0b, 0xe4, 0x16, 0x44, 0x42, 0xb2, 0x00, 0x00, 0xe1, 0x16, 0x45, 0x66]));
        expect(encode('tlv', '/3303/0', {5700: 31, 5701: 'c'})).to.be.eql(cutils.bufferFrom([0x08, 0x00, 0x0b, 0xe4, 0x16, 0x44, 0x41, 0xf8, 0x00, 0x00, 0xe1, 0x16, 0x45, 0x63]));
        expect(encode('tlv', '/3303/0/5700', 31)).to.be.eql(cutils.bufferFrom([0xe4, 0x16, 0x44, 0x41, 0xf8, 0x00, 0x00]));
        expect(encode('tlv', '/temperature', {0: {5700: 31, 5701: 'c'}, 1:{5700: 89, 5701: 'f'}})).to.be.eql(cutils.bufferFrom([0x08, 0x00, 0x0b, 0xe4, 0x16, 0x44, 0x41, 0xf8, 0x00, 0x00, 0xe1, 0x16, 0x45, 0x63, 0x08, 0x01, 0x0b, 0xe4, 0x16, 0x44, 0x42, 0xb2, 0x00, 0x00, 0xe1, 0x16, 0x45, 0x66]));
        expect(encode('tlv', '/temperature/0', {5700: 31, 5701: 'c'})).to.be.eql(cutils.bufferFrom([0x08, 0x00, 0x0b, 0xe4, 0x16, 0x44, 0x41, 0xf8, 0x00, 0x00, 0xe1, 0x16, 0x45, 0x63]));
        expect(encode('tlv', '/3303/0/sensorValue', 31)).to.be.eql(cutils.bufferFrom([0xe4, 0x16, 0x44, 0x41, 0xf8, 0x00, 0x00]));
        expect(encode('tlv', '/temperature/0/sensorValue', 31)).to.be.eql(cutils.bufferFrom([0xe4, 0x16, 0x44, 0x41, 0xf8, 0x00, 0x00]));
        expect(encode('tlv', '/3303/0', {sensorValue: 31, units: 'c'})).to.be.eql(cutils.bufferFrom([0x08, 0x00, 0x0b, 0xe4, 0x16, 0x44, 0x41, 0xf8, 0x00, 0x00, 0xe1, 0x16, 0x45, 0x63]));
     });

    it('#.encode(json)', function () {
        expect(encode('json', 'x', { 1: {  0: 'x', 1: 5 }, 2: {  0: true, 1: 0 }})).to.be.eql(cutils.bufferFrom(JSON.stringify({ bn: '/x', e: [{ n: '1/0', sv: 'x' }, { n: '1/1', v: 5 }, { n: '2/0', bv: true }, { n:'2/1', v: 0}] })));
        expect(encode('json', 'x/y', { 0: 'x', 1: 5, 2: new Date(100000) })).to.be.eql(cutils.bufferFrom(JSON.stringify({ bn: '/x/y', e: [{ n: '0', sv: 'x' }, { n: '1', v: 5 }, { n: '2', v: 100000 }] })));
        expect(encode('json', 'x/y/z', 5)).to.be.eql(cutils.bufferFrom(JSON.stringify({ bn: '/x/y/z', e: [{ n: '', v: 5}]})));
        expect(encode('json', 'x/y/z', new Date(100000))).to.be.eql(cutils.bufferFrom(JSON.stringify({ bn: '/x/y/z', e: [{ n: '', v: 100000}]})));
    });
});