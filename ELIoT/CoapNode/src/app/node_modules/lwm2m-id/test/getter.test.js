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

describe('Getters Check', function () {
    describe('#RspCode.get', function () {
        it('should get right keys', function () {
            rspCodeKeys.forEach(function (key) {
                var item = lwm2mid.RspCode.get(key),
                    itemData = lwm2mid._defs.rspCode,
                    val = item.value;

                expect(item).not.to.be.undefined;
                expect(val).to.be.eql(itemData[key]);

                item = lwm2mid.RspCode.get(val);

                expect(item).not.to.be.undefined;
                expect(item.key).to.be.eql(key);

            });
        });

        it('should get right values', function () {
            rspCodeVals.forEach(function (val) {
                var item = lwm2mid.RspCode.get(val),
                    itemData = lwm2mid._defs.rspCode,
                    key = item.key;

                expect(item).not.to.be.undefined;
                expect(val).to.be.eql(itemData[key]);
                item = lwm2mid.RspCode.get(key);
                expect(item).not.to.be.undefined;
                expect(item.value).to.be.eql(val);
            });
        });
    });

    describe('#Cmd.get', function () {
        it('should get right keys', function () {
            cmdIdKeys.forEach(function (key) {
                var item = lwm2mid.Cmd.get(key),
                    itemData = lwm2mid._defs.cmdId,
                    val = item.value;

                expect(item).not.to.be.undefined;
                expect(val).to.be.eql(itemData[key]);

                item = lwm2mid.Cmd.get(val);
                expect(item).not.to.be.undefined;
                expect(item.key).to.be.eql(key);
            });
        });

        it('should get right values', function () {
            cmdIdVals.forEach(function (val) {
                var item = lwm2mid.Cmd.get(val),
                    itemData = lwm2mid._defs.cmdId,
                    key = item.key;

                expect(item).not.to.be.undefined;
                expect(val).to.be.eql(itemData[key]);

                item = lwm2mid.Cmd.get(key);
                expect(item).not.to.be.undefined;
                expect(item.value).to.be.eql(val);
            });
        });
    });

    describe('#Oid.get', function () {
        it('should get right keys', function () {
            oidKeys.forEach(function (key) {
                var item = lwm2mid.Oid.get(key),
                    itemData = lwm2mid._defs.oid,
                    val = item.value;

                expect(item).not.to.be.undefined;
                expect(val).to.be.eql(itemData[key]);

                item = lwm2mid.Oid.get(val);
                expect(item).not.to.be.undefined;
                expect(item.key).to.be.eql(key);
            });
        });

        it('should get right values', function () {
            oidVals.forEach(function (val) {
                var item = lwm2mid.Oid.get(val),
                    itemData = lwm2mid._defs.oid,
                    key = item.key;

                expect(item).not.to.be.undefined;
                expect(val).to.be.eql(itemData[key]);

                item = lwm2mid.Oid.get(key);
                expect(item).not.to.be.undefined;
                expect(item.value).to.be.eql(val);
            });
        });
    });

    describe('#UniqueRid.get', function () {
        it('should get right keys', function () {
            uRidKeys.forEach(function (key) {
                var item = lwm2mid.UniqueRid.get(key),
                    itemData = lwm2mid._defs.uniqueRid,
                    val = item.value;

                expect(item).not.to.be.undefined;
                expect(val).to.be.eql(itemData[key]);

                item = lwm2mid.UniqueRid.get(val);
                expect(item).not.to.be.undefined;
                expect(item.key).to.be.eql(key);
            });
        });

        it('should get right values', function () {
            uRidVals.forEach(function (val) {
                var item = lwm2mid.UniqueRid.get(val),
                    itemData = lwm2mid._defs.uniqueRid,
                    key = item.key;

                expect(item).not.to.be.undefined;
                expect(val).to.be.eql(itemData[key]);

                item = lwm2mid.UniqueRid.get(key);
                expect(item).not.to.be.undefined;
                expect(item.value).to.be.eql(val);
            });
        });
    });

    describe('#SpecificRid[x].get', function () {
        it('should get right keys and values', function () {
            for (var sk in sRidKeys) {
                sRidKeys[sk].forEach(function (key) {
                    var item = lwm2mid.SpecificRid[sk].get(key),
                        itemData = lwm2mid._defs.specificRid[sk],
                        val = item.value;

                    expect(item).not.to.be.undefined;
                    expect(val).to.be.eql(itemData[key]);

                    item = lwm2mid.SpecificRid[sk].get(val);
                    expect(item).not.to.be.undefined;
                    expect(item.key).to.be.eql(key);
                });

                sRidVals[sk].forEach(function (val) {
                    var item = lwm2mid.SpecificRid[sk].get(val),
                        itemData = lwm2mid._defs.specificRid[sk],
                        key = item.key;

                    expect(item).not.to.be.undefined;
                    expect(val).to.be.eql(itemData[key]);

                    item = lwm2mid.SpecificRid[sk].get(key);
                    expect(item).not.to.be.undefined;
                    expect(item.value).to.be.eql(val);
                });
            }
        });
    });

    describe('#getOid', function () {
        it('should get right item by oid string', function () {
            oidKeys.forEach(function (okey) {
                var hitA = lwm2mid.getOid(okey),
                    hitB = lwm2mid.Oid.get(okey);

                expect(hitA).not.to.be.undefined;
                expect(hitA.key).to.be.eql(hitB.key);
                expect(hitA.value).to.be.eql(hitB.value);
            });
        });

        it('should get right item by oid number', function () {
            oidVals.forEach(function (oval) {
                var hitA = lwm2mid.getOid(oval),
                    hitB = lwm2mid.Oid.get(oval);

                expect(hitA).not.to.be.undefined;
                expect(hitA.key).to.be.eql(hitB.key);
                expect(hitA.value).to.be.eql(hitB.value);
            });
        });

        it('should get undefined if oid not found', function () {
            expect(lwm2mid.getOid('xxx')).to.be.undefined;
            expect(lwm2mid.getOid(12345)).to.be.undefined;
        });
    });

    describe('#getRid - from UniqueRid', function () {
        it('should get right item by rid string', function () {
            uRidKeys.forEach(function (rkey) {
                var hitA = lwm2mid.getRid(rkey),
                    hitB = lwm2mid.UniqueRid.get(rkey);

                expect(hitA).not.to.be.undefined;
                expect(hitA.key).to.be.eql(hitB.key);
                expect(hitA.value).to.be.eql(hitB.value);
            });
        });

        it('should get right item by rid number', function () {
            uRidVals.forEach(function (rval) {
                var hitA = lwm2mid.getRid(rval),
                    hitB = lwm2mid.UniqueRid.get(rval);

                expect(hitA).not.to.be.undefined;
                expect(hitA.key).to.be.eql(hitB.key);
                expect(hitA.value).to.be.eql(hitB.value);
            });
        });

        it('should get undefined if rid not found', function () {
            expect(lwm2mid.getRid('xxx')).to.be.undefined;
            expect(lwm2mid.getRid(12345)).to.be.undefined;
        });
    });


    describe('#getRid - from SpecificRid', function () {
        it('should get right item by oid string and rid string', function () {
            sOidKeys.forEach(function (okey) {
                sRidKeys[okey].forEach(function (rkey) {
                    var hitA = lwm2mid.getRid(okey, rkey),
                        hitB = lwm2mid.SpecificRid[okey].get(rkey);

                    expect(hitA).not.to.be.undefined;
                    expect(hitA.key).to.be.eql(hitB.key);
                    expect(hitA.value).to.be.eql(hitB.value);
                });
            });
        });

        it('should get right item by oid string and rid number', function () {
            sOidKeys.forEach(function (okey) {
                sRidVals[okey].forEach(function (rval) {
                    var hitA = lwm2mid.getRid(okey, rval),
                        hitB = lwm2mid.SpecificRid[okey].get(rval);

                    expect(hitA).not.to.be.undefined;
                    expect(hitA.key).to.be.eql(hitB.key);
                    expect(hitA.value).to.be.eql(hitB.value);
                });
            });
        });


        it('should get right item by oid number and rid string', function () {
            sOidVals.forEach(function (oval) {
                var okey = lwm2mid.getOid(oval).key;

                sRidKeys[okey].forEach(function (rkey) {
                    var hitA = lwm2mid.getRid(oval, rkey),
                        hitB = lwm2mid.SpecificRid[okey].get(rkey);

                    expect(hitA).not.to.be.undefined;
                    expect(hitA.key).to.be.eql(hitB.key);
                    expect(hitA.value).to.be.eql(hitB.value);
                });
            });
        });

        it('should get right item by oid number and rid number', function () {
            sOidVals.forEach(function (oval) {
                var okey = lwm2mid.getOid(oval).key;

                sRidVals[okey].forEach(function (rval) {
                    var hitA = lwm2mid.getRid(oval, rval),
                        hitB = lwm2mid.SpecificRid[okey].get(rval);

                    expect(hitA).not.to.be.undefined;
                    expect(hitA.key).to.be.eql(hitB.key);
                    expect(hitA.value).to.be.eql(hitB.value);
                });
            });
        });

        it('should get undefined if target not found', function () {
            expect(lwm2mid.getRid('device', 'dddd')).to.be.undefined;
            expect(lwm2mid.getRid('device', 12345)).to.be.undefined;
            expect(lwm2mid.getRid(3, 'dddd')).to.be.undefined;
            expect(lwm2mid.getRid(3, 12345)).to.be.undefined;
        });

        it('should get an item if target is found', function () {
            expect(lwm2mid.getRid('device', 'reboot')).not.to.be.undefined;
            expect(lwm2mid.getRid('device', 4)).not.to.be.undefined;
            expect(lwm2mid.getRid(3, 'reboot')).not.to.be.undefined;
            expect(lwm2mid.getRid(3, 4)).not.to.be.undefined;
        });
    });

    describe('getOdef', function () {
        it('should get right spec', function () {
            oidKeys.forEach(function (key) {
                var item = lwm2mid.getOid(key),
                    itemData = lwm2mid._defs.oid,
                    val = item.value,
                    spec;

                expect(item).not.to.be.undefined;
                expect(val).to.be.eql(itemData[key]);

                spec = lwm2mid.getOdef(key);

                if (val >= 3200 && val <= 3400)
                    expect(spec).to.be.deep.eql({ multi: true, mand: false });
                else if (key === 'lwm2mSecurity')
                    expect(spec).to.be.deep.eql({ multi: true, mand: true });
                else if (key === 'lwm2mServer')
                    expect(spec).to.be.deep.eql({ multi: true, mand: true });
                else if (key === 'accessCtrl')
                    expect(spec).to.be.deep.eql({ multi: true, mand: false });
                else if (key === 'device')
                    expect(spec).to.be.deep.eql({ multi: false, mand: true });
                else if (key === 'connMonitor')
                    expect(spec).to.be.deep.eql({ multi: false, mand: false });
                else if (key === 'firmware')
                    expect(spec).to.be.deep.eql({ multi: false, mand: false });
                else if (key === 'location')
                    expect(spec).to.be.deep.eql({ multi: false, mand: false });
                else if (key === 'connStatistics')
                    expect(spec).to.be.deep.eql({ multi: false, mand: false });
                else 
                    expect(spec).to.be.undefined;

                item = lwm2mid.Oid.get(val);
                expect(item).not.to.be.undefined;
                expect(item.key).to.be.eql(key);

                spec = lwm2mid.getOdef(val);
                if (val >= 3200 && val <= 3400)
                    expect(spec).to.be.deep.eql({ multi: true, mand: false });
                else if (key === 'lwm2mSecurity')
                    expect(spec).to.be.deep.eql({ multi: true, mand: true });
                else if (key === 'lwm2mServer')
                    expect(spec).to.be.deep.eql({ multi: true, mand: true });
                else if (key === 'accessCtrl')
                    expect(spec).to.be.deep.eql({ multi: true, mand: false });
                else if (key === 'device')
                    expect(spec).to.be.deep.eql({ multi: false, mand: true });
                else if (key === 'connMonitor')
                    expect(spec).to.be.deep.eql({ multi: false, mand: false });
                else if (key === 'firmware')
                    expect(spec).to.be.deep.eql({ multi: false, mand: false });
                else if (key === 'location')
                    expect(spec).to.be.deep.eql({ multi: false, mand: false });
                else if (key === 'connStatistics')
                    expect(spec).to.be.deep.eql({ multi: false, mand: false });
                else 
                    expect(spec).to.be.undefined;
            });
        });

    });

    describe('#getSpecificResrcChar', function () {
        it('should get right char by oid string and rid string', function () {
            sOidCharKeys.forEach(function (okey) {
                sRidKeys[okey].forEach(function (skey) {
                    expect(lwm2mid.getSpecificResrcChar(okey, skey)).to.be.eql(lwm2mid.SpecificResrcChar[okey][skey]);
                });
            });
        });

        it('should get right char by oid number and rid string', function () {
            sOidCharVals.forEach(function (oval) {
                var okey = lwm2mid.getOid(oval).key;

                sRidKeys[okey].forEach(function (skey) {
                    var sid = lwm2mid.getRid(oval, skey).value;
                    expect(lwm2mid.getSpecificResrcChar(oval, sid)).to.be.eql(lwm2mid.SpecificResrcChar[okey][skey]);
                });
            });
        });
    });
});
