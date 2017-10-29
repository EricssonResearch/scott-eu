'use strict';

var _ = require('busyman'),
    lwm2mId = require('lwm2m-id');

var cutils = require('./cutils');

function encode(type, path, value, attrs) {
    switch (type) {
        case 'link':
            return encodeLink(path, value, attrs);

        case 'tlv':
            return encodeTlv(path, value);

        case 'json':
            return encodeJson(path, value);

        default:
            break;
    }
}

/*********************************************************
 * Link-format                                           *
 *********************************************************/
function encodeLink(path, value, attrs) {
    var allowedAttrs = [ 'pmin', 'pmax', 'gt', 'lt', 'st' ],
        pathType = cutils.getPathDateType(path),
        pathArray = cutils.getPathArray(path),
        oid = pathArray[0],
        attrsPayload = '',
        linkFormat = '';

    _.forEach(attrs, function (val, key) {
        if (_.includes(allowedAttrs, key) && _.isNumber(val))
            attrsPayload = attrsPayload + ';' + key + '=' + val;   // ';pmin=0;pmax=60'
    });

    path = cutils.getNumPath(path);
    linkFormat = '<' + path + '>' + attrsPayload + ',';

    switch (pathType) {
        case 'object':         // obj
            _.forEach(value, function (iobj, iid) {
                _.forEach(iobj, function (val, rid) {
                    linkFormat = linkFormat + '<' + path + '/' + iid + '/' + cutils.ridNumber(oid, rid) + '>' + ',';
                });
            });
            break;

        case 'instance':         // inst
            _.forEach(value, function (val, rid) {
                linkFormat = linkFormat + '<' + path + '/' + cutils.ridNumber(oid, rid) + '>' + ',';
            });
            break;

        case 'resource':         // resrc
            break;

        default:
            break;
    }

    if (linkFormat[linkFormat.length-1] === ',')
        linkFormat = linkFormat.slice(0, linkFormat.length - 1);

    return linkFormat;
}

/*********************************************************
 * Tlv                                                   *
 *********************************************************/
function encodeTlv(path, value) {
    var pathType = cutils.getPathDateType(path),
        pathArray,
        data;

    path = cutils.getNumPath(path);
    pathArray = cutils.getPathArray(path);

    switch (pathType) {
        case 'object':
            data = encodeTlvPacket('object', path, pathArray[0], value);
            break;

        case 'instance':
            data = encodeTlvPacket('instance', path, pathArray[1], value);
            break;

        case 'resource':
            if (_.isPlainObject(value)) {
                data = encodeTlvPacket('multiResrc', path, pathArray[2], value);
            } else {
                data = encodeTlvPacket('resource', path, pathArray[2], value);
            }
            break;

        default:
            break;
    }

    return data;
}

function encodeTlvPacket(type, path, id, value) {
    var oid = cutils.getPathArray(path)[0],
        resrcDef,
        resrcType,
        dataBuf,
        valBuf;

    switch (type) {
        case 'object':
            valBuf = [];

            _.forEach(value, function (iObj, iid) {
                valBuf.push(encodeTlvPacket('instance', path, iid, iObj));
            });

            dataBuf = Buffer.concat(valBuf);
            break;
        case 'instance':
            valBuf = [];

            _.forEach(value, function (resrc, rid) {
                rid = cutils.ridNumber(oid, rid);

                if (_.isPlainObject(resrc)) {
                    valBuf.push(encodeTlvPacket('multiResrc', path, rid, resrc));
                } else {
                    valBuf.push(encodeTlvPacket('resource', path, rid, resrc));
                }
            });

            dataBuf = encodeTlvBuf(type, id, Buffer.concat(valBuf));
            break;

        case 'multiResrc':
            valBuf = [];

            _.forEach(value, function (r, riid) {
                valBuf.push(encodeTlvPacket('resrcInst', path, riid, r));
            });

            dataBuf = encodeTlvBuf(type, id, Buffer.concat(valBuf));
            break;

        case 'resource':
        case 'resrcInst':
            resrcDef = lwm2mId.getRdef(oid, id);
            resrcType = resrcDef ? resrcDef.type : undefined;
            valBuf = encodeTlvResrc(resrcType, value);
            dataBuf = encodeTlvBuf(type, id, valBuf);
            break;

        default:
            break;
    }

    return dataBuf;
}

function encodeTlvBuf(type, id, valBuf) {
    var dataBuf,
        typeBuf, idBuf, lenBuf,
        len = valBuf.length,    // Bytes
        byte = 0x00,
        idType;

/***************************
 *  TYPE                   *
 ***************************/
    switch (type) {
        case 'instance':
            byte = byte | 0x00;
            break;
        case 'resource':
            byte = byte | 0xC0;
            break;
        case 'multiResrc':
            byte = byte | 0x80;
            break;
        case 'resrcInst':
            byte = byte | 0x40;
            break;
    }

    // [TODO] id should be number
    if (id >= 256) {
        byte = byte | 0x20;
        idType = 'uint16be';
    } else {
        byte = byte | 0x00;
        idType = 'uint8';
    }

    // [TODO] else Error
    if (len < 8)
        byte = byte | len;
    else if (len < 256)
        byte = byte | 0x08;
    else if (len < 65536)
        byte = byte | 0x10;
    else if (len < 16777215)
        byte = byte | 0x18;

    typeBuf = cutils.bufferAlloc(1);
    typeBuf.writeUInt8(byte);

/***************************
 *  ID                     *
 ***************************/
    if (idType === 'uint16be') {
        idBuf = cutils.bufferAlloc(2);
        idBuf.writeUInt16BE(id);
    } else {
        idBuf = cutils.bufferAlloc(1);
        idBuf.writeUInt8(id);
    }

/***************************
 *  LENGTH                 *
 ***************************/
    if (len >= 8 && len < 256) {
        lenBuf = cutils.bufferAlloc(1);
        lenBuf.writeUInt8(len);
    } else if (len >= 256 && len < 65536) {          // [TODO] value is out of bounds
        lenBuf = cutils.bufferAlloc(2);
        lenBuf.writeUInt16BE(len);
    } else if (len >= 65536 && len < 16777216) {     // [TODO] value is out of bounds
        lenBuf = cutils.bufferAlloc(3);
        lenBuf.writeUInt8((len >> 16) & 0xFF);
        lenBuf.writeUInt16BE(len & 0xFF, 1);
    } else {
        lenBuf = cutils.bufferFrom([]);
    }

/***************************
 *  VALUE                  *
 ***************************/
    dataBuf = Buffer.concat([typeBuf, idBuf, lenBuf, valBuf]);

    return dataBuf;
}

function encodeTlvResrc(resrcType, value) {
    var dataBuf,
        type, len, zLen;

    if (!resrcType) {
        if (_.isBoolean(value)) {
            resrcType = 'boolean';
        } else if (_.isNumber(value)) {
            if (_.isInteger(value))
                resrcType = 'integer';
            else
                resrcType = 'float';
        } else if (_.isString(value)) {
            resrcType = 'string';
        }
    }

    switch (resrcType) {
        case 'boolean':
            dataBuf = cutils.bufferAlloc(1);
            dataBuf.writeUInt8(value ? 1 : 0);
            break;

        case 'time':
        case 'integer':
            len = cutils.length(value);

            if (len == 1) {
                dataBuf = cutils.bufferAlloc(1);
                dataBuf.writeInt8(value);
            } else if (len == 2) {
                dataBuf = cutils.bufferAlloc(2);
                dataBuf.writeInt16BE(value);
            } else if (len <= 4) {
                dataBuf = cutils.bufferAlloc(4);
                dataBuf.writeInt32BE(value);
            } else if (len <= 8) {
                dataBuf = cutils.bufferAlloc(8);
                dataBuf.writeInt32BE(value / 0xFFFFFFFF, 0);
                dataBuf.writeInt32BE(value & 0xFFFFFFFF, 4);
            } else {
                // [TODO] Error?
            }
            break;

        // [TODO] double
        case 'float':
            dataBuf = cutils.bufferAlloc(4);
            dataBuf.writeFloatBE(value);
            break;

        case 'string':
        case 'execute':
            dataBuf = cutils.bufferFrom(value, 'utf8');
            break;

        // [TODO] Objlnk
        default:
            dataBuf = cutils.bufferFrom('_unreadable_', 'utf8');
            break;
    }

    return dataBuf;
}

/*********************************************************
 * Json                                                  *
 *********************************************************/
function encodeJson(path, value) {
    var objInJson = {
            bn: cutils.getNumPath(path),
            e: []
        },
        pathType = cutils.getPathDateType(path),
        pathArray = cutils.getPathArray(path),
        oid = pathArray[0];

    if (pathType !== 'resource' && !_.isPlainObject(value))
        throw new TypeError('value should be an object.');

    switch (pathType) {
        case 'object':         // obj
            _.forEach(value, function (iObj, iid) {
                _.forEach(iObj, function (resrc, rid) {
                    if (_.isPlainObject(resrc)) {
                        _.forEach(resrc, function (r, riid) {
                            var data = encodeJsonValue(iid + '/' + cutils.ridNumber(oid, rid) + '/' + riid, r);
                            objInJson.e.push(data);
                        });
                    } else {
                        var data = encodeJsonValue(iid + '/' + cutils.ridNumber(oid, rid), resrc);
                        objInJson.e.push(data);
                    }
                });
            });
            break;

        case 'instance':         // inst
            _.forEach(value, function (resrc, rid) {
                if (_.isPlainObject(resrc)) {
                    _.forEach(resrc, function (r, riid) {
                        var data = encodeJsonValue(cutils.ridNumber(oid, rid) + '/' + riid, r);
                        objInJson.e.push(data);
                    });
                } else {
                    var data = encodeJsonValue(cutils.ridNumber(oid, rid), resrc);
                    objInJson.e.push(data);
                }
            });
            break;

        case 'resource':         // resrc
            if (_.isPlainObject(value)) {
                _.forEach(value, function (r, riid) {
                    var data = encodeJsonValue(riid, r);
                    objInJson.e.push(data);
                });
            } else {
                if (value instanceof Date) value = Number(value);
                var data = encodeJsonValue('', value);
                objInJson.e.push(data);
            }
            break;

        default:
            break;
     }

    return  cutils.bufferFrom(JSON.stringify(objInJson));
}

function encodeJsonValue(path, value) {
    var val = {};

    if (_.isNil(value)) {
        value = path;
        path = undefined;
    } else {
        val.n = path.toString();
    }

    if (_.isNumber(value)) {
        val.v = Number(value);
    } else if (_.isString(value)) {
        val.sv = String(value);
    } else if (value instanceof Date) {
        val.v = Number(value);
    } else if (_.isBoolean(value)) {
        val.bv = Boolean(value);
    } else if (_.isPlainObject(value)) {
        val.ov = value;     // [TODO] objlnk
    }

    return val;
}

/*********************************************************
 * Module Exports                                        *
 *********************************************************/
module.exports = encode;
