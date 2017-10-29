'use strict';

var urlParser = require('url').parse,
    lwm2mId = require('lwm2m-id'),
	_ = require('busyman');

var cutils = {};

cutils.length = function (val) {
    var len, bytes, 
        isNegative = false; 

    if (val < 0) {
        val = Math.abs(val);
        isNegative = true;
    }

    if (val < 1) 
        return 1;

    bytes = Math.log2(val + 1) / 8;
    len = Math.ceil(bytes);

    if (isNegative && _.isInteger(bytes))
        len = len + 1;

    return len;		// Bytes
};

/*********************************************************
 * Buffer                                                *
 *********************************************************/
if (Object.prototype.hasOwnProperty.call(Buffer, 'alloc')) {
    cutils.bufferAlloc = Buffer.alloc;
} else {
    cutils.bufferAlloc = function (size) {
        return new Buffer(size);
    };
}

if (Object.prototype.hasOwnProperty.call(Buffer, 'from')) {
    cutils.bufferFrom = Buffer.from;
} else {
    cutils.bufferFrom = function (data, encoding) {
        return new Buffer(data, encoding);
    };
}

/*********************************************************
 * lwm2m-id utils                                        *
 *********************************************************/
 cutils.oidKey = function (oid) {
    var oidItem = lwm2mId.getOid(oid);
    return oidItem ? oidItem.key : oid;
};

cutils.oidNumber = function (oid) {
    var oidItem = lwm2mId.getOid(oid);

    oidItem = oidItem ? oidItem.value : parseInt(oid);   

    if (_.isNaN(oidItem))
        oidItem = oid;

    return oidItem;
};

cutils.ridKey = function (oid, rid) {
    var ridItem = lwm2mId.getRid(oid, rid);

    if (_.isUndefined(rid))
        rid = oid;

    return ridItem ? ridItem.key : rid;
};

cutils.ridNumber = function (oid, rid) {
    var ridItem = lwm2mId.getRid(oid, rid);

    if (_.isUndefined(rid))
        rid = oid;

    ridItem = ridItem ? ridItem.value : parseInt(rid);   

    if (_.isNaN(ridItem))
        ridItem = rid;

    return ridItem;
};

/*********************************************************
 * path utils                                            *
 *********************************************************/
cutils.checkPathBegin = function (path) {
    if (path.charAt(0) === '/') {
        return path;
    } else {
        return '/' + path;
    }
};

cutils.getPathArray = function (path) {		// '/x/y/z'
    var pathArray = urlParser(path).pathname.split('/');

    if (pathArray[0] === '') 
        pathArray = pathArray.slice(1);

    if (pathArray[pathArray.length-1] === '')           
        pathArray = pathArray.slice(0, pathArray.length-1);

    return pathArray;  // ['x', 'y', 'z']
};

cutils.getPathDateType = function (path) {
    var pathArray = this.getPathArray(path),
        dateType = [ 'so', 'object', 'instance', 'resource' ][pathArray.length];
    return dateType;
};

cutils.getNumPath = function (path) {
    var pathArray = this.getPathArray(path),       // '/lwm2mServer/2/defaultMaxPeriod'
        numPath = '',
        oid,
        rid;

    if (pathArray[0]) {    //oid
        oid = this.oidNumber(pathArray[0]);
        numPath += '/' + oid;

        if (pathArray[1]) {    //iid
            numPath += '/' + pathArray[1]; 

            if (pathArray[2]) {    //rid
                rid = this.ridNumber(oid, pathArray[2]);
                numPath +=  '/' + rid;
            } 
        }
    }

    return numPath;      // '/1/2/3'
};

cutils.getPathIdKey = function (path) {		// '/x/y/z'
    var pathArray = this.getPathArray(path),
        pathObj = {},
        oid,
        rid;

    if (path) {
        if (pathArray[0]) {    //oid
            oid = this.oidKey(pathArray[0]);
            pathObj.oid = oid;
            if (pathArray[1]) {    //iid
                pathObj.iid = + pathArray[1]; 
                if (pathArray[2]) {    //rid
                    rid = this.ridKey(oid, pathArray[2]);
                    pathObj.rid = rid;
                } 
            }
        }
    }

    return pathObj;     // {oid:'lwm2mServer', iid: '2', rid: 'defaultMaxPeriod'}
};

module.exports = cutils;
