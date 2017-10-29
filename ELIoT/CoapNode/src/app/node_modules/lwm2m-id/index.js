'use strict';

var Enum = require('enum'),
    _defs = require('./defs/defs.json'),
    _specificRid = _defs.specificRid,
    DEFS = {
        _Enum: Enum,
        _defs: _defs,
        RspCode: null,
        Cmd: null,
        Oid: null,
        UniqueRid: null,
        SpecificRid: {},
        SpecificResrcChar: _defs.specificResrcChar,
        objectSpec: _defs.objectSpec
    };

/*************************************************************************************************/
/*** Loading Enumerations                                                                      ***/
/*************************************************************************************************/
DEFS.RspCode = new Enum(_defs.rspCode);
DEFS.Cmd = new Enum(_defs.cmdId);
DEFS.Oid = new Enum(_defs.oid);
DEFS.UniqueRid = new Enum(_defs.uniqueRid);

for (var key in _specificRid) {
    if (_specificRid.hasOwnProperty(key))
        DEFS.SpecificRid[key] = new Enum(_specificRid[key]);
}

function isValidArgType(param) {
    var isValid = true;

    if (typeof param !== 'number' && typeof param !== 'string') {
        isValid = false;
    } else if (typeof param === 'number') {
        isValid = !isNaN(param);
    }

    return isValid;
}

/*************************************************************************************************/
/*** DEFS Methods                                                                              ***/
/*************************************************************************************************/
DEFS.getCmd = function (cmdId) {
    if (!isValidArgType(cmdId))
        throw new TypeError('cmdId should be type of string or number.');

    return DEFS.Cmd.get(cmdId);
};

DEFS.getRspCode = function (code) {
    if (!isValidArgType(code))
        throw new TypeError('code should be a type of string or number.');

    return DEFS.RspCode.get(code);
};

DEFS.getOid = function (oid) {
    if (!isValidArgType(oid))
        throw new TypeError('oid should be a number or a string.');

    var oidNumber = parseInt(oid),
        oidItem;

    if (!isNaN(oidNumber))
        oid = oidNumber;

    oidItem = DEFS.Oid.get(oid);

    return oidItem;
};

DEFS.addOid = function (items) {
    var _oid = DEFS._defs.oid;

    if (typeof items !== 'object' || items === null || Array.isArray(items))
        throw new TypeError('items should be a plain object.');

    for (var key in items) {
        if (DEFS.getOid(key))
            throw new Error('oid: ' + key + ' name conflicts.');
        else if (DEFS.getOid(items[key]))
            throw new Error('oid: ' + key + ' value conflicts.');
        else
            _oid[key] = items[key];
    }

    DEFS.Oid = null;
    DEFS.Oid = new Enum(_oid);

    return DEFS;
};

DEFS.getRid = function (oid, rid) {
    var oidItem = DEFS.getOid(oid),
        ridNumber,
        ridItem,
        oidKey;

    if (typeof rid === 'undefined') {
        if (typeof oid === 'undefined')
            throw new TypeError('Bad arguments');

        rid = oid;
        oid = undefined;
        if (!isValidArgType(rid))
            throw new TypeError('rid should be a number or a string.');
    }

    ridNumber = parseInt(rid);
    if (!isNaN(ridNumber))
        rid = ridNumber;

    if (typeof oid !== 'undefined') {           // searching in MDEFS.RIDOFOID
        if (!isValidArgType(oid))
            throw new TypeError('rid should be a number or a string.');

        if (typeof rid === 'undefined')
            throw new TypeError('rid should be given');

        if (!isValidArgType(rid))
            throw new TypeError('rid should be a number or a string.');

        oidKey = oidItem ? oidItem.key : oid.toString();

        if (DEFS.SpecificRid[oidKey] instanceof Enum)
            ridItem = DEFS.SpecificRid[oidKey].get(rid);
    } else {
        ridItem = DEFS.UniqueRid.get(rid);
    }

    return ridItem;
};

DEFS.addUniqueRid = function (items) {
    var _uRid = DEFS._defs.uniqueRid;

    if (typeof items !== 'object' || items === null || Array.isArray(items))
        throw new TypeError('items should be a plain object.');

    for (var key in items) {
        if (DEFS.getRid(key))
            throw new Error('unique rid: ' + key + ' name conflicts.');
        else if (DEFS.getRid(items[key]))
            throw new Error('unique rid: ' + key + ' value conflicts.');
        else
            _uRid[key] = items[key];
    }

    DEFS.UniqueRid = null;
    DEFS.UniqueRid = new Enum(_uRid);

    return DEFS;
};

DEFS.addSpecificRid = function (oid, items) {
    var oidItem = DEFS.getOid(oid),
        oidKey,
        ridItem, 
        _spfRid = DEFS._defs.specificRid;

    if (!oidItem)
        throw new Error('oid: ' + oid + ' does not exist. Please do addOid() first.');

    oidKey = oidItem.key;

    _spfRid[oidKey] = _spfRid[oidKey] || {};

    if (typeof items !== 'object' || items === null || Array.isArray(items))
        throw new TypeError('items should be a plain object.');

    for (var key in items) {
        if (typeof _spfRid[oidKey][key] !== 'undefined') {
            throw new Error('rid: ' + key + ' within oid: ' + oidKey + ' conflicts.');
        }

        _spfRid[oidKey][key] = items[key];
    }

    DEFS.SpecificRid[oidKey] = null;
    DEFS.SpecificRid[oidKey] = new Enum(_spfRid[oidKey]);

    return DEFS;
};

DEFS.getOdef = function (oid) {
    var oidItem = DEFS.getOid(oid),
        spec;

    if (!oidItem)
        return;

    spec = DEFS.objectSpec[oidItem.key];

    // 3200-3400: defined by starter and expansion pack
    if (!spec && oidItem.value >= 3200 && oidItem.value <= 3400)
        spec = { multi: true, mand: false };

    return spec;
};  // undefined / Object spec.

DEFS.getSpecificResrcChar = function (oid, rid) {
    var oidItem = DEFS.getOid(oid),
        ridItem = DEFS.getRid(oid, rid),
        characteristic;

    if (!ridItem)
        return;

    if (oidItem && ridItem) {
        characteristic = DEFS.SpecificResrcChar[oidItem.key];
        characteristic = characteristic[ridItem.key];
    }

    return characteristic;
};  // undefined / resrc characteristic

DEFS.getRdef = DEFS.getSpecificResrcChar;

DEFS.addSpecificResrcChar = function (oid, chars) {
    var _rChar = DEFS._defs.specificResrcChar,
        oidItem = DEFS.getOid(oid),
        ridItem,
        pass = _checkCharFormat(chars);

    if (!oidItem)
        throw new Error('oid: ' + oid + ' does not exist. Please do addOid() first.');

    _rChar[oidItem.key] = _rChar[oidItem.key] || {};
    _rChar = _rChar[oidItem.key];

    if (typeof chars !== 'object' || chars === null || Array.isArray(chars))
        throw new TypeError('chars should be a plain object.');

    for (var rid in chars) {
        ridItem = DEFS.getRid(oid, rid);
        if (!ridItem)
            throw new Error('rid: ' + rid + ' does not exist. Please do addSpecificRid() first.');

        if (_rChar[ridItem.key]) {
            throw new Error('rid: ' + rid + ' conflicts in oid: ' + oid);
        } else {
            if (!_checkCharFormat(chars[rid]))
                throw new Error('Invalid characteristic format within rid: ' + rid);

             _rChar[ridItem.key] = chars[rid];
        }
    }

    return DEFS;
};

/*************************************************************************************************/
/*** Private Functions                                                                         ***/
/*************************************************************************************************/
function _checkCharFormat(charItem) {
    var keysChecked = {
        access: false,
        multi: false,
        mand: false,
        type: false,
        range: false,
        init: false
    },
    pass = true;

    if (typeof charItem !== 'object')
        throw new TypeError('Resource characteristic should be an object.');

    for (var key in charItem) {
        if (keysChecked.hasOwnProperty(key))
            keysChecked[key] = true;
    }

    for (var k in keysChecked) {
        pass = pass & keysChecked[k];
    }

    return pass;
}

module.exports = DEFS;
