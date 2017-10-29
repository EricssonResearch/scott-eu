'use strict';

var _ = require('busyman'),
    utils = require('./utils'),
    lwm2mId = require('lwm2m-id'),
    ObjInstance = require('./object_instance');

function SmartObject(hal, setup) {
    if (_.isFunction(hal)) {
        setup = hal;
        hal = undefined;
    }

    if (!_.isUndefined(setup) && !_.isFunction(setup))
        throw new TypeError('setup should be a function if given.');

    hal = hal || {};
    Object.defineProperty(this, 'hal', { value: hal, writable: true, enumerable: false, configurable: false });
    Object.defineProperty(this, 'ipsoOnly', { value: false, writable: true, enumerable: false, configurable: false });

    if (_.isFunction(setup))
        setup.call(this);
}

/*************************************************************************************************/
/*** Public Methods: Smart Object Initialization                                               ***/
/*************************************************************************************************/
SmartObject.prototype.init = function (oid, iid, resrcs, setup) {
    var objInst;

    objInst = this.findObjectInstance(oid, iid) || this.create(oid, iid);

    // init() will auto create namespace, and will firstly clear all resources in the Object Instance
    objInst.init(resrcs, setup);
    return objInst;
};

SmartObject.prototype.create = function (oid, iid) {
    var oidKey,
        objInst;

    if (!utils.isValidArgType(iid))
        throw new TypeError('iid should be given with a number or a string.');

    oidKey = utils.getOidKey(oid);
    this[oidKey] = this[oidKey] || {};

    if (!_.isNil(this[oidKey][iid]))
        throw new Error('Object instance of iid ' + iid + ' already exists. Cannot create.');

    if (this.ipsoOnly) {
        if (!_.isNumber(iid))
            throw new TypeError('iid must be a number (ipsoOnly).');
        else if (!utils.isIpsoOid(oid))
            throw new Error('oid not in an IPSO-defined Object.');
    }

    objInst = this[oidKey][iid] = new ObjInstance(oidKey, iid, this);

    return objInst;
};

/*************************************************************************************************/
/*** Public Methods: Synchronous                                                               ***/
/*************************************************************************************************/
SmartObject.prototype.remove = function (oid, iid) {
    var objInst;

    objInst = this.findObjectInstance(oid, iid);

    if (objInst) {
        objInst.clear();
        objInst = null;
        delete this.findObject(oid)[iid];
        return true;
    }

    return false;
};

SmartObject.prototype.objectList = function () {
    var objList = [];

    _.forEach(this, function (obj, oidKey) {
        var iidList = [];

        if (!_.isObject(obj))
            return;

        _.forEach(obj, function (inst, iid) {
            var iidNum = parseInt(iid);
            iidNum = _.isNaN(iidNum) ? iid : iidNum;

            if (_.isObject(inst))
                iidList.push(iidNum);
        });

        objList.push({ oid: utils.getOidNum(oidKey), iid: iidList });
    });
    return objList;
};

SmartObject.prototype.has = function (oid, iid, rid) {
    var oidKey = utils.getOidKey(oid),
        obj = this[oidKey],
        has = !!obj,
        objInst,
        resrc;

    if (!_.isUndefined(iid) && !utils.isValidArgType(iid)) 
        throw new TypeError('iid should be given with a number or a string.');

    if (has && !_.isUndefined(iid)) {
        objInst = obj[iid];
        has = !!objInst;
        if (has && !_.isUndefined(rid)) {
            has = objInst.has(rid);
        }
    }
    return has;
};

SmartObject.prototype.findObject = function (oid) {
    return this[utils.getOidKey(oid)];
};

SmartObject.prototype.findObjectInstance = function (oid, iid) {
    if (!utils.isValidArgType(iid)) 
        throw new TypeError('iid should be given with a number or a string.');

    var target = this.findObject(oid);
    return target ? target[iid] : target;
};

SmartObject.prototype.isReadable = function (oid, iid, rid) {
    var rsc = this.get(oid, iid, rid),
        readable = false,
        rdef;

    if (_.isUndefined(rsc))
        return readable;

    if (!_.isPlainObject(rsc) || !rsc._isCb) {
        rdef = lwm2mId.getRdef(oid, rid);
        readable = (!rdef) || (rdef.access === 'R' || rdef.access === 'RW');
    } else if (_.isFunction(rsc.read)) {
        readable = true;
    }
    return readable;
};

SmartObject.prototype.isWritable = function (oid, iid, rid) {
    var rsc = this.get(oid, iid, rid),
        writable = false,
        rdef;

    if (_.isUndefined(rsc))
        return writable;

    if (!_.isPlainObject(rsc) || !rsc._isCb) {
        rdef = lwm2mId.getRdef(oid, rid);
        writable = (!rdef) || (rdef.access === 'W' || rdef.access === 'RW');
    } else if (_.isFunction(rsc.write)) {
        writable = true;
    }
    return writable;
};

SmartObject.prototype.isExecutable = function (oid, iid, rid) {
    var rsc = this.get(oid, iid, rid),
        executable = false;

    if (_.isUndefined(rsc))
        return executable;

    executable = _.isObject(rsc) && _.isFunction(rsc.exec);
    return executable;
};

SmartObject.prototype.get = function (oid, iid, rid) {
    var objInst = this.findObjectInstance(oid, iid);
    return objInst ? objInst.get(rid) : undefined;
};

SmartObject.prototype.set = function (oid, iid, rid, value) {
    var objInst = this.findObjectInstance(oid, iid),
        set = !!objInst;

    if (objInst)
        objInst.set(rid, value);

    return set;
};

SmartObject.prototype.dumpSync = function (oid, iid) {
    var dumpType = [ 'so', 'obj', 'objInst' ][arguments.length],
        target;

    if (!dumpType)
        throw new Error('Bad arguments. What do you like to dump?');

    if (dumpType === 'objInst') {
        target = this.findObjectInstance(oid, iid);
        return target ? target.dumpSync() : undefined;
    }

    if (dumpType === 'obj') {
        target = this.findObject(oid);
        return target ? utils.dumpObjectSync(target) : undefined;
    }

    if (dumpType === 'so') {
        var dumped = {};
        _.forEach(this, function (o, oidKey) {
            dumped[oidKey] = utils.dumpObjectSync(o);
        });
        return dumped;
    }
};

/*************************************************************************************************/
/*** Public Methods: Asynchronous                                                              ***/
/*************************************************************************************************/
SmartObject.prototype.dump = function (oid, iid, opt, callback) {
    var dumpType = 'so',
        dumped = {},
        target;

    if (arguments.length === 1) {                               // (callback)
        callback = oid;
        dumpType = 'so';
    } else if (arguments.length === 2) {
        if (_.isPlainObject(oid)) {                             // (opt, callback)
            opt = oid;
            callback = iid;
            dumpType = 'so';
        } else {                                                // (oid, callback)
            callback = iid;
            dumpType = 'obj';
        }
    } else if (arguments.length === 3) {
        if (_.isPlainObject(iid)) {                             // (oid, opt, callback)
            callback = opt;
            opt = iid;
            dumpType = 'obj';
        } else {                                                // (oid, iid, callback)
            callback = opt;
            opt = undefined;
            dumpType = 'objInst';
        }
    } else if (arguments.length === 4) {                        // (oid, iid, opt, callback)
        dumpType = 'objInst';
    } else {
        throw new Error('Bad arguments. What do you like to dump? Do you give me a callback?');
    }

    if (!_.isFunction(callback))
        throw new TypeError('Callback should be a function.');

    if (dumpType === 'objInst') {
        target = this.findObjectInstance(oid, iid);
        if (target)
            target.dump(opt, callback);
    } else if (dumpType === 'obj') {
        target = this.findObject(oid);
        if (target)
            utils.dumpObject(target, opt, callback);
    } else if (dumpType === 'so') {
        target = this;
        utils.dumpSmartObject(target, opt, callback);
    }

    if (!target)
        utils.invokeCbNextTick(new Error('Target not found. Cannot dump.'), null, callback);

    return this;
};

SmartObject.prototype.read = function (oid, iid, rid, opt, callback) {
    var objInst = this.findObjectInstance(oid, iid);
    return objInst ? objInst.read(rid, opt, callback) : utils.invokeCbNextTick(new Error('Object or Object Instance not found.'), '_notfound_', callback);
};

SmartObject.prototype.write = function (oid, iid, rid, value, opt, callback) {
    var objInst = this.findObjectInstance(oid, iid);
    return objInst ? objInst.write(rid, value, opt, callback) : utils.invokeCbNextTick(new Error('Object or Object Instance not found.'), '_notfound_', callback);
};

SmartObject.prototype.exec = function (oid, iid, rid, argus, callback) {
    var objInst = this.findObjectInstance(oid, iid);
    return objInst ? objInst.exec(rid, argus, callback) : utils.invokeCbNextTick(new Error('Object or Object Instance not found.'), '_notfound_', callback);
};

module.exports = SmartObject;
