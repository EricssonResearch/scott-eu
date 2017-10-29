var Enum = require('enum'),
    expect = require('chai').expect,
    lwm2mid = require('../index.js');     // lwm2m-id module

describe('Enum Instance Check', function() {
    describe('#._defs', function() {
        it('is not an object of Enum', function () {
            expect(lwm2mid._defs instanceof Enum).to.be.false;
        });
    });

    describe('#.RspCode', function() {
        it('is an object of Enum', function () {
            expect(lwm2mid.RspCode instanceof Enum).to.be.true;
        });
    });

    describe('#.Cmd', function() {
        it('is an object of Enum', function () {
            expect(lwm2mid.Cmd instanceof Enum).to.be.true;
        });
    });

    describe('#.Cmd', function() {
        it('is an object of Enum', function () {
            expect(lwm2mid.Cmd instanceof Enum).to.be.true;
        });
    });

    describe('#.Oid', function() {
        it('is an object of Enum', function () {
            expect(lwm2mid.Oid instanceof Enum).to.be.true;
        });
    });

    describe('#.UniqueRid', function() {
        it('is an object of Enum', function () {
            expect(lwm2mid.UniqueRid instanceof Enum).to.be.true;
        });
    });

    describe('#.SpecificRid', function() {
        it('are objects of Enum', function () {
            var pass = true;
            for (var oid in lwm2mid.SpecificRid) {
                pass = pass && (lwm2mid.SpecificRid[oid] instanceof Enum);
            }
            expect(pass).to.be.true;
        });

    });

    describe('#.specificResrcChar', function() {
        it('is not an object of Enum', function () {
            expect(lwm2mid.specificResrcChar instanceof Enum).to.be.false;
        });
    });
});

describe('APIs Arguments Check for Throwing Errors', function() {

    describe('#.getCmd', function() {
        it('should be a function', function () {
            expect(lwm2mid.getCmd).to.be.a('function');
        });

        it('should throw TypeError if input cmdId is not a number and not a string', function () {
            expect(function () { return lwm2mid.getCmd(); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getCmd(undefined); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getCmd(null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getCmd(NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getCmd([]); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getCmd({}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getCmd(true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getCmd(new Date()); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getCmd(function () {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getCmd(3); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getCmd('3'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getCmd('xx'); }).not.to.throw(Error);
        });
    });

    describe('#.getRspCode', function() {
        it('should be a function', function () {
            expect(lwm2mid.getRspCode).to.be.a('function');
        });

        it('should throw TypeError if input code is not a number and not a string', function () {
            expect(function () { return lwm2mid.getRspCode(); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRspCode(undefined); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRspCode(null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRspCode(NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRspCode([]); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRspCode({}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRspCode(true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRspCode(new Date()); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRspCode(function () {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getRspCode(3); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRspCode('3'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRspCode('xx'); }).not.to.throw(Error);
        });
    });

    describe('#.getOid', function() {
        it('should be a function', function () {
            expect(lwm2mid.getOid).to.be.a('function');
        });

        it('should throw TypeError if input oid is not a number and not a string', function () {
            expect(function () { return lwm2mid.getOid(); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getOid(undefined); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getOid(null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getOid(NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getOid([]); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getOid({}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getOid(true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getOid(new Date()); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getOid(function () {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getOid(3); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getOid('3'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getOid('xx'); }).not.to.throw(Error);
        });
    });

    describe('#.getRid', function() {
        it('should be a function', function () {
            expect(lwm2mid.getRid).to.be.a('function');
        });

        it('should throw TypeError if input oid is not a number and not a string', function () {
            expect(function () { return lwm2mid.getRid(undefined, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(undefined, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(null, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(NaN, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid([], 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid({}, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(true, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(new Date(), 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(function () {}, 3); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getRid(undefined, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(undefined, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(null, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(NaN, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid([], '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid({}, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(true, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(new Date(), '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(function () {}, '3'); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getRid(undefined, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(undefined, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(null, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(NaN, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid([], 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid({}, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(true, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(new Date(), 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(function () {}, 'xx'); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getRid(3, 3); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRid('3', 3); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRid('xx', 3); }).not.to.throw(Error);

            expect(function () { return lwm2mid.getRid(3, '3'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRid('3', '3'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRid('xx', 'xx'); }).not.to.throw(Error);
        });

        it('should throw TypeError if input rid is not a number and not a string', function () {
            expect(function () { return lwm2mid.getRid(3, null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(3, NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(3, []); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(3, {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(3, true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(3, new Date()); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(3, function () {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getRid('3', null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid('3', NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid('3', []); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid('3', {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid('3', true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid('3', new Date()); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid('3', function () {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getRid('xx', null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid('xx', NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid('xx', []); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid('xx', {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid('xx', true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid('xx', new Date()); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRid('xx', function () {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getRid(3); }).not.to.throw(TypeError);
            expect(function () { return lwm2mid.getRid('3'); }).not.to.throw(TypeError);
            expect(function () { return lwm2mid.getRid(3, 3); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRid(3, '3'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRid(3, 'xx'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRid('xx'); }).not.to.throw(TypeError);
            expect(function () { return lwm2mid.getRid('3', 3); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRid('3', '3'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRid('xx', 'xx'); }).not.to.throw(Error);
        });
    });

    describe('#.getSpecificResrcChar', function() {
        it('should be a function', function () {
            expect(lwm2mid.getSpecificResrcChar).to.be.a('function');
        });

        it('should throw TypeError if input oid is not a number and not a string', function () {
            expect(function () { return lwm2mid.getSpecificResrcChar(undefined, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(undefined, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(null, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(NaN, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar([], 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar({}, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(true, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(new Date(), 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(function () {}, 3); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getSpecificResrcChar(undefined, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(undefined, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(null, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(NaN, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar([], '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar({}, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(true, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(new Date(), '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(function () {}, '3'); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getSpecificResrcChar(undefined, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(undefined, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(null, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(NaN, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar([], 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar({}, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(true, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(new Date(), 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(function () {}, 'xx'); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getSpecificResrcChar(3, 3); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getSpecificResrcChar('3', 3); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getSpecificResrcChar('xx', 3); }).not.to.throw(Error);

            expect(function () { return lwm2mid.getSpecificResrcChar(3, '3'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getSpecificResrcChar('3', '3'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getSpecificResrcChar('xx', 'xx'); }).not.to.throw(Error);
        });

        it('should throw TypeError if input rid is not a number and not a string', function () {
            expect(function () { return lwm2mid.getSpecificResrcChar(3, null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(3, NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(3, []); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(3, {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(3, true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(3, new Date()); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(3, function () {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getSpecificResrcChar('3', null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar('3', NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar('3', []); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar('3', {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar('3', true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar('3', new Date()); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar('3', function () {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getSpecificResrcChar('xx', null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar('xx', NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar('xx', []); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar('xx', {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar('xx', true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar('xx', new Date()); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar('xx', function () {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getSpecificResrcChar(3); }).not.to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar('3'); }).not.to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar(3, 3); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getSpecificResrcChar(3, '3'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getSpecificResrcChar(3, 'xx'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getSpecificResrcChar('xx'); }).not.to.throw(TypeError);
            expect(function () { return lwm2mid.getSpecificResrcChar('3', 3); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getSpecificResrcChar('3', '3'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getSpecificResrcChar('xx', 'xx'); }).not.to.throw(Error);
        });
    });

    describe('#.getOdef', function() {
        it('should be a function', function () {
            expect(lwm2mid.getOdef).to.be.a('function');
        });

        it('should throw TypeError if input oid is not a number and not a string', function () {
            expect(function () { return lwm2mid.getOdef(); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getOdef(undefined); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getOdef(null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getOdef(NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getOdef([]); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getOdef({}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getOdef(true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getOdef(new Date()); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getOdef(function () {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getOdef(3); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getOdef('3'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getOdef('xx'); }).not.to.throw(Error);
        });
    });

    describe('#.getRdef', function() {
        it('should be a function', function () {
            expect(lwm2mid.getRdef).to.be.a('function');
        });

        it('should throw TypeError if input oid is not a number and not a string', function () {
            expect(function () { return lwm2mid.getRdef(undefined, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(undefined, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(null, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(NaN, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef([], 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef({}, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(true, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(new Date(), 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(function () {}, 3); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getRdef(undefined, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(undefined, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(null, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(NaN, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef([], '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef({}, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(true, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(new Date(), '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(function () {}, '3'); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getRdef(undefined, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(undefined, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(null, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(NaN, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef([], 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef({}, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(true, 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(new Date(), 'xx'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(function () {}, 'xx'); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getRdef(3, 3); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRdef('3', 3); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRdef('xx', 3); }).not.to.throw(Error);

            expect(function () { return lwm2mid.getRdef(3, '3'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRdef('3', '3'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRdef('xx', 'xx'); }).not.to.throw(Error);
        });

        it('should throw TypeError if input rid is not a number and not a string', function () {
            expect(function () { return lwm2mid.getRdef(3, null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(3, NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(3, []); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(3, {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(3, true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(3, new Date()); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(3, function () {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getRdef('3', null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef('3', NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef('3', []); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef('3', {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef('3', true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef('3', new Date()); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef('3', function () {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getRdef('xx', null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef('xx', NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef('xx', []); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef('xx', {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef('xx', true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef('xx', new Date()); }).to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef('xx', function () {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.getRdef(3); }).not.to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef('3'); }).not.to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef(3, 3); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRdef(3, '3'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRdef(3, 'xx'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRdef('xx'); }).not.to.throw(TypeError);
            expect(function () { return lwm2mid.getRdef('3', 3); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRdef('3', '3'); }).not.to.throw(Error);
            expect(function () { return lwm2mid.getRdef('xx', 'xx'); }).not.to.throw(Error);
        });
    });

    describe('#.addOid', function() {
        it('should be a function', function () {
            expect(lwm2mid.addOid).to.be.a('function');
        });

        it('should throw TypeError if input oid is not a number and not a string', function () {
            expect(function () { return lwm2mid.addOid(); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addOid(undefined); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addOid(null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addOid(NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addOid([]); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addOid(true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addOid(function () {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addOid(3); }).to.throw(Error);
            expect(function () { return lwm2mid.addOid('3'); }).to.throw(Error);
            expect(function () { return lwm2mid.addOid('xx'); }).to.throw(Error);

            expect(function () { return lwm2mid.addOid({}); }).not.to.throw(TypeError);
        });
    });
    
    describe('#.addUniqueRid', function() {
        it('should be a function', function () {
            expect(lwm2mid.addUniqueRid).to.be.a('function');
        });


        it('should throw TypeError if input oid is not a number and not a string', function () {
            expect(function () { return lwm2mid.addUniqueRid(); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addUniqueRid(undefined); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addUniqueRid(null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addUniqueRid(NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addUniqueRid([]); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addUniqueRid(true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addUniqueRid(function () {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addUniqueRid(3); }).to.throw(Error);
            expect(function () { return lwm2mid.addUniqueRid('3'); }).to.throw(Error);
            expect(function () { return lwm2mid.addUniqueRid('xx'); }).to.throw(Error);

            expect(function () { return lwm2mid.addUniqueRid({}); }).not.to.throw(TypeError);
        });
    });

    describe('#.addSpecificRid', function() {
        it('should be a function', function () {
            expect(lwm2mid.addSpecificRid).to.be.a('function');
        });

        it('should throw TypeError if input oid is not a number and not a string', function () {
            expect(function () { return lwm2mid.addSpecificRid(undefined, {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid(null, {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid(NaN, {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid([], {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid({}, {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid(true, {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid(new Date(), {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid(function () {}, {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.addSpecificRid(3, {}); }).not.to.throw(Error);
            expect(function () { return lwm2mid.addSpecificRid('3', {}); }).not.to.throw(Error);
        });

        it('should throw TypeError if input items is not an object', function () {
            expect(function () { return lwm2mid.addSpecificRid(3, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid('3', '3'); }).to.throw(TypeError);

            expect(function () { return lwm2mid.addSpecificRid('3', '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid('3', []); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid('3', undefined); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid('3', null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid('3', NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid('3', true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid('3', function () {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.addSpecificRid(3, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid(3, []); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid(3, undefined); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid(3, null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid(3, NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid(3, true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificRid(3, function () {}); }).to.throw(TypeError);
        });
    });

    describe('#.addSpecificResrcChar', function() {
        it('should be a function', function () {
            expect(lwm2mid.addSpecificResrcChar).to.be.a('function');
        });

        it('should throw TypeError if input oid is not a number and not a string', function () {
            expect(function () { return lwm2mid.addSpecificResrcChar(undefined, {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar(null, {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar(NaN, {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar([], {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar({}, {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar(true, {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar(new Date(), {}); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar(function () {}, {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.addSpecificResrcChar(3, {}); }).not.to.throw(Error);
            expect(function () { return lwm2mid.addSpecificResrcChar('3', {}); }).not.to.throw(Error);
        });

        it('should throw TypeError if input chars is not an object', function () {
            expect(function () { return lwm2mid.addSpecificResrcChar(3, 3); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar('3', '3'); }).to.throw(TypeError);

            expect(function () { return lwm2mid.addSpecificResrcChar('3', '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar('3', []); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar('3', undefined); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar('3', null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar('3', NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar('3', true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar('3', function () {}); }).to.throw(TypeError);

            expect(function () { return lwm2mid.addSpecificResrcChar(3, '3'); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar(3, []); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar(3, undefined); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar(3, null); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar(3, NaN); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar(3, true); }).to.throw(TypeError);
            expect(function () { return lwm2mid.addSpecificResrcChar(3, function () {}); }).to.throw(TypeError);
        });
    });
});
