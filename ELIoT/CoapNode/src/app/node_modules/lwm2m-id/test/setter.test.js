var Enum = require('enum'),
    expect = require('chai').expect,
    lwm2mid = require('../index.js');     // lwm2m-id module

var rspCodeKeys = [],
    rspCodeVals = [],
    cmdIdKeys = [],
    cmdIdVals = [],
    oidKeys = [],
    oidVals = [],
    uRidKeys = [],
    uRidVals = [],
    sOidKeys = [],
    sOidVals = [],
    sRidKeys = {},
    sRidVals = {},
    sOidCharKeys = [],
    sOidCharVals = [],
    k;

for (k in lwm2mid._defs.rspCode) {
    rspCodeKeys.push(k);
    rspCodeVals.push(lwm2mid._defs.rspCode[k]);
}

for (k in lwm2mid._defs.cmdId) {
    cmdIdKeys.push(k);
    cmdIdVals.push(lwm2mid._defs.cmdId[k]);
}

for (k in lwm2mid._defs.oid) {
    oidKeys.push(k);
    oidVals.push(lwm2mid._defs.oid[k]);
}

for (k in lwm2mid._defs.uniqueRid) {
    uRidKeys.push(k);
    uRidVals.push(lwm2mid._defs.uniqueRid[k]);
}

for (k in lwm2mid._defs.specificRid) {
    sOidKeys.push(k);
    sOidVals.push(lwm2mid._defs.oid[k]);
    sRidKeys[k] = [];
    sRidVals[k] = [];
    for (var i in lwm2mid._defs.specificRid[k]) {
        sRidKeys[k].push(i);
        sRidVals[k].push(lwm2mid._defs.specificRid[k][i]);
    }
}

for (k in lwm2mid._defs.specificResrcChar) {
    sOidCharKeys.push(k);
    sOidCharVals.push(lwm2mid.Oid.get(k).value);
}

describe('Setters Check', function () {
    describe('#addOid', function () {
        it('should get properly after add a new oid', function () {
            lwm2mid.addOid({ 'test1': 1001, 'test2': 1002 });

            expect(lwm2mid.getOid('test1').key).to.be.eql('test1');
            expect(lwm2mid.getOid('test2').key).to.be.eql('test2');
            expect(lwm2mid.getOid('test1').value).to.be.eql(1001);
            expect(lwm2mid.getOid('test2').value).to.be.eql(1002);

            expect(lwm2mid.getOid('1001').key).to.be.eql('test1');
            expect(lwm2mid.getOid('1002').key).to.be.eql('test2');
            expect(lwm2mid.getOid('1001').value).to.be.eql(1001);
            expect(lwm2mid.getOid('1002').value).to.be.eql(1002);

            expect(lwm2mid.getOid(1001).key).to.be.eql('test1');
            expect(lwm2mid.getOid(1002).key).to.be.eql('test2');
            expect(lwm2mid.getOid(1001).value).to.be.eql(1001);
            expect(lwm2mid.getOid(1002).value).to.be.eql(1002);
        });


        it('should throw if oid string conficts when adding', function () {
            expect(function () { return lwm2mid.addOid({ "cmdhLimits": 1234 }); }).to.throw(Error);
        });

        it('should throw if oid number conficts when adding', function () {
            expect(function () { return lwm2mid.addOid({ "xxxxx": 2053 }); }).to.throw(Error);
        });
    });

    describe('#addUniqueRid', function () {
        it('should get properly after add a new rid', function () {
            lwm2mid.addUniqueRid({ 'rtest1': 1001, 'rtest2': 1002 });


            expect(lwm2mid.getRid('rtest1').key).to.be.eql('rtest1');
            expect(lwm2mid.getRid('rtest2').key).to.be.eql('rtest2');
            expect(lwm2mid.getRid('rtest1').value).to.be.eql(1001);
            expect(lwm2mid.getRid('rtest2').value).to.be.eql(1002);

            expect(lwm2mid.getRid('1001').key).to.be.eql('rtest1');
            expect(lwm2mid.getRid('1002').key).to.be.eql('rtest2');
            expect(lwm2mid.getRid('1001').value).to.be.eql(1001);
            expect(lwm2mid.getRid('1002').value).to.be.eql(1002);

            expect(lwm2mid.getRid(1001).key).to.be.eql('rtest1');
            expect(lwm2mid.getRid(1002).key).to.be.eql('rtest2');
            expect(lwm2mid.getRid(1001).value).to.be.eql(1001);
            expect(lwm2mid.getRid(1002).value).to.be.eql(1002);
        });

        it('should throw if rid string conficts when adding', function () {
            expect(function () { return lwm2mid.addUniqueRid({ "edgeSelection": 1234 }); }).to.throw(Error);
        });

        it('should throw if rid number conficts when adding', function () {
            expect(function () { return lwm2mid.addUniqueRid({ "xxxxx": 5504 }); }).to.throw(Error);
        });
    });

    describe('#addSpecificRid', function () {
        it('should get properly after add a new rid', function () {
            lwm2mid.addSpecificRid('lwm2mServer', { 'srTest1': 1001, 'srTest2': 1002 });

            expect(lwm2mid.getRid('lwm2mServer', 'srTest1').key).to.be.eql('srTest1');
            expect(lwm2mid.getRid('lwm2mServer', 'srTest2').key).to.be.eql('srTest2');
            expect(lwm2mid.getRid('lwm2mServer', 'srTest1').value).to.be.eql(1001);
            expect(lwm2mid.getRid('lwm2mServer', 'srTest2').value).to.be.eql(1002);

            expect(lwm2mid.getRid('lwm2mServer', '1001').key).to.be.eql('srTest1');
            expect(lwm2mid.getRid('lwm2mServer', '1002').key).to.be.eql('srTest2');
            expect(lwm2mid.getRid('lwm2mServer', '1001').value).to.be.eql(1001);
            expect(lwm2mid.getRid('lwm2mServer', '1002').value).to.be.eql(1002);

            expect(lwm2mid.getRid('lwm2mServer', 1001).key).to.be.eql('srTest1');
            expect(lwm2mid.getRid('lwm2mServer', 1002).key).to.be.eql('srTest2');
            expect(lwm2mid.getRid('lwm2mServer', 1001).value).to.be.eql(1001);
            expect(lwm2mid.getRid('lwm2mServer', 1002).value).to.be.eql(1002);

        });

        it('should throw if no such oid', function () {
            expect(function() { return lwm2mid.addSpecificRid('xyz', { 'xyzTest1': 1001, 'xyzTest2': 1002 }); }).to.throw(Error);
        });

        it('should not throw if there is the oid found', function () {
            lwm2mid.addOid({ 'xyz': 9999 });
            expect(function() { return lwm2mid.addSpecificRid('xyz', { 'xyzTest1': 1001, 'xyzTest2': 1002 }); }).not.to.throw(Error);
        });

        it('should get properly after add a new oid', function () {
            expect(lwm2mid.getRid('xyz', 'xyzTest1').key).to.be.eql('xyzTest1');
            expect(lwm2mid.getRid('xyz', 'xyzTest2').key).to.be.eql('xyzTest2');
            expect(lwm2mid.getRid('xyz', 'xyzTest1').value).to.be.eql(1001);
            expect(lwm2mid.getRid('xyz', 'xyzTest2').value).to.be.eql(1002);

            expect(lwm2mid.getRid('xyz', '1001').key).to.be.eql('xyzTest1');
            expect(lwm2mid.getRid('xyz', '1002').key).to.be.eql('xyzTest2');
            expect(lwm2mid.getRid('xyz', '1001').value).to.be.eql(1001);
            expect(lwm2mid.getRid('xyz', '1002').value).to.be.eql(1002);

            expect(lwm2mid.getRid('xyz', 1001).key).to.be.eql('xyzTest1');
            expect(lwm2mid.getRid('xyz', 1002).key).to.be.eql('xyzTest2');
            expect(lwm2mid.getRid('xyz', 1001).value).to.be.eql(1001);
            expect(lwm2mid.getRid('xyz', 1002).value).to.be.eql(1002);

            expect(lwm2mid.getRid(9999, 'xyzTest1').key).to.be.eql('xyzTest1');
            expect(lwm2mid.getRid(9999, 'xyzTest2').key).to.be.eql('xyzTest2');
            expect(lwm2mid.getRid(9999, 'xyzTest1').value).to.be.eql(1001);
            expect(lwm2mid.getRid(9999, 'xyzTest2').value).to.be.eql(1002);

            expect(lwm2mid.getRid(9999, '1001').key).to.be.eql('xyzTest1');
            expect(lwm2mid.getRid(9999, '1002').key).to.be.eql('xyzTest2');
            expect(lwm2mid.getRid(9999, '1001').value).to.be.eql(1001);
            expect(lwm2mid.getRid(9999, '1002').value).to.be.eql(1002);

            expect(lwm2mid.getRid(9999, 1001).key).to.be.eql('xyzTest1');
            expect(lwm2mid.getRid(9999, 1002).key).to.be.eql('xyzTest2');
            expect(lwm2mid.getRid(9999, 1001).value).to.be.eql(1001);
            expect(lwm2mid.getRid(9999, 1002).value).to.be.eql(1002);
        });
    });

    describe('#addSpecificResrcChar', function () {
        it('should throw if no such oid when add', function () {
            expect(function () {
                lwm2mid.addSpecificResrcChar('abc', {
                    "xValue": { "access": "R", "multi": false, "mand": true, "type": "float", "range": null, "init": 0 },
                    "yValue": { "access": "R", "multi": false, "mand": false, "type": "float", "range": null, "init": 0 },
                    "zValue": { "access": "R", "multi": false, "mand": false, "type": "float", "range": null, "init": 0 },
                    "units": { "access": "R", "multi": false, "mand": false, "type": "string", "range": null, "init": "uint" },
                    "compassDir": { "access": "R", "multi": false, "mand": false, "type": "float", "range": 360, "init": 0 }
                });
            }).to.throw(Error);
        });

        it('should throw if rids do not exist under a specifc oid when add', function () {
            lwm2mid.addOid({ abc: 9998 });
            expect(function () {
                return lwm2mid.addSpecificResrcChar('abc', {
                    "xValue": { "access": "R", "multi": false, "mand": true, "type": "float", "range": null, "init": 0 },
                    "yValue": { "access": "R", "multi": false, "mand": false, "type": "float", "range": null, "init": 0 },
                });
            }).to.throw(Error);
        });

        it('should throw if a rid does not exist under a specifc oid when add', function () {
            lwm2mid.addSpecificRid(9998, { xValue: 3000, yValue: 3001 });
            expect(function () {
                return lwm2mid.addSpecificResrcChar('abc', {
                    "xValue": { "access": "R", "multi": false, "mand": true, "type": "float", "range": null, "init": 0 },
                    "yValue": { "access": "R", "multi": false, "mand": false, "type": "float", "range": null, "init": 0 },
                    "zValue": { "access": "R", "multi": false, "mand": false, "type": "float", "range": null, "init": 0 },
                });
            }).to.throw(Error);
        });

        it('should throw if one of rids exists under a specifc oid when add', function () {
            expect(function () {
                return lwm2mid.addSpecificRid(9998, {
                    xValue: 3000,
                    yValue: 3001,
                    zValue: 3002,
                });
            }).to.throw(Error);
        });

        it('should not throw if a rid does not exist under a specifc oid when add', function () {
            expect(function () {
                return lwm2mid.addSpecificRid(9998, {
                    zValue: 3002
                });
            }).not.to.throw(Error);
        });

        it('should not throw if a rid exists under a specifc oid when add', function () {
            expect(function () {
                return lwm2mid.addSpecificResrcChar('abc', {
                    "zValue": { "access": "R", "multi": false, "mand": false, "type": "float", "range": null, "init": 0 }
                });
            }).not.to.throw(Error);
        });

        it('should throw if a rid exists under a specifc oid when add again', function () {
            expect(function () {
                return lwm2mid.addSpecificResrcChar('abc', {
                    "zValue": { "access": "R", "multi": false, "mand": false, "type": "float", "range": null, "init": 0 }
                });
            }).to.throw(Error);
        });

        it('should not throw if a rid does under a specifc oid when add', function () {
            expect(function () {
                return lwm2mid.addSpecificRid('setPoint', {
                    zValue: 3002,
                });
            }).not.to.throw(Error)
        });

        it('should not throw if a rid does under a specifc oid when add', function () {
            expect(function () {
                return lwm2mid.addSpecificResrcChar('setPoint', {
                    "zValue": { "access": "R", "multi": false, "mand": false, "type": "float", "range": null, "init": 0 },
                });
            }).not.to.throw(Error);
        });

        it('should get properly after addSpecificResrcChar succefully ', function () {
            expect(lwm2mid.getSpecificResrcChar('setPoint', 3002)).to.be.eql({
               "access": "R", "multi": false, "mand": false, "type": "float", "range": null, "init": 0 
            });
        });
    });
});
