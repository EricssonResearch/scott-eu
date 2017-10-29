'use strict';

var _ = require('busyman'),
    lwm2mId = require('lwm2m-id');

var cutils = require('./cutils');

function decode(type, path, value) {
    if (path && !value) {
        value = path;
        path = null;
    }

    switch (type) {
        case 'link':
            return decodeLink(value);

        case 'tlv':
            return decodeTlv(path, value);

        case 'json':
            return decodeJson(path, value);

        default:
            break;
    }
}

/*********************************************************
 * Link-format                                           *
 *********************************************************/
function decodeLink(value) {                                // '</1/2>;pmin=10;pmax=60,</1/2/1>,</1/2/2>'
    var allowedAttrs = [ 'pmin', 'pmax', 'gt', 'lt', 'st' ],
        valueArray = value.split(','),                      // ['</1/2>;pmin=10;pmax=60', '</1/2/1>', '</1/2/2>']
        resrcArray = valueArray.slice(1),                   // ['</1/2/1>', '</1/2/2>']
        attrsArray = valueArray[0].split(';').slice(1),     // ['pmin=10', 'pmax=60']
        path = valueArray[0].split(';')[0].slice(1, -1),
        data = {
            path: path
        },
        rid;    

    if (!_.isEmpty(attrsArray)) {
        data.attrs = {};
        _.forEach(attrsArray, function (val) {
            var attr = val.split('=');

            if (_.includes(allowedAttrs, attr[0]))
                data.attrs[attr[0]] = Number(attr[1]);
        });
    }

    if (!_.isEmpty(resrcArray)) {
        data.resrcList = [];
        _.forEach(resrcArray, function (resrc, idx) {     
            data.resrcList.push(resrc.slice(1, -1));  
        });
    }

    return data;     // { path: '/1/2', attrs: { pmin: 10, pmax: 60 }, resrcList: ['/1/2/1', '/1/2/2'] }
}

/*********************************************************
 * Tlv                                                   *
 *********************************************************/
function decodeTlv(path, value) {
    var data = {},
        offset = 0,
        len = value.length,
        pathType = cutils.getPathDateType(path),
        pathArray = cutils.getPathArray(path),
        bufData,
        type,
        oid = pathArray[0],
        iid,
        rid;

    while (offset < len) {
        bufData = decodeTlvBuf(path, value, offset);
        type = bufData.type;
        offset = bufData.offset;

        switch (pathType) {
            case 'object':
                if (type === 'instance') {
                    iid = bufData.id;
                    data[iid] = {};
                } else if (type === 'resource') {
                    rid = cutils.ridKey(oid, bufData.id);
                    data[iid][rid] = bufData.value;
                } else if (type === 'multiResrc') {
                    rid = cutils.ridKey(oid, bufData.id);
                    data[rid] = {};
                } else if (type === 'resrcInst') {
                    data[iid][rid][bufData.id] = bufData.value;
                }
                break;

            case 'instance':
                if (type === 'instance') {
                    iid = bufData.id;
                } else if (type === 'resource') {
                    rid = cutils.ridKey(oid, bufData.id);
                    data[rid] = bufData.value;
                } else if (type === 'multiResrc') {
                    rid = cutils.ridKey(oid, bufData.id);
                    data[rid] = {};
                } else if (type === 'resrcInst') {
                    data[rid][bufData.id] = bufData.value;
                }
                break;

            case 'resource':
                if (type === 'resource') {
                    data = bufData.value;
                } else if (type === 'multiResrc') {
                    rid = cutils.ridKey(oid, bufData.id);
                } else if (type === 'resrcInst') {
                    data[bufData.id] = bufData.value;
                }
                break;

            default:
                break;
        }

    }

    return data;
}

function decodeTlvBuf(path, value, offset) {
    var typeBuf, valBuf,
        type, id, len, val,
        data = {};

    offset = offset || 0;
    typeBuf = value[offset];

/***************************
 *  TYPE                   * 
 ***************************/
    if ((typeBuf & 0xC0) === 0xC0) {
        type = 'resource';
    } else if ((typeBuf & 0x80) === 0x80) {
        type = 'multiResrc';
    } else if ((typeBuf & 0x40) === 0x40) {
        type = 'resrcInst';
    } else {
        type = 'instance';
    }

    offset = offset + 1;

/***************************
 *  ID                     * 
 ***************************/
    if ((typeBuf & 0x20) === 0x20) {
        id = value.readUInt16BE(offset);
        offset = offset + 2;
    } else {
        id = value.readUInt8(offset);
        offset = offset + 1;
    }

/***************************
 *  LENGTH                 * 
 ***************************/
    if ((typeBuf & 0x18) === 0x18) {
        len = value.readUInt8(offset) * 65536 + value.readUInt16BE(offset + 1);
        offset = offset + 3;
    } else if ((typeBuf & 0x10) === 0x10) {
        len = value.readUInt16BE(offset);
        offset = offset + 2;
    } else if ((typeBuf & 0x08) === 0x08) {
        len = value.readUInt8(offset);
        offset = offset + 1;
    } else {
        len = typeBuf & 0x07;
    }

/***************************
 *  VALUE                  * 
 ***************************/
    switch (type) {
        case 'instance':
            break;

        case 'multiResrc':
            break;

        case 'resource':
        case 'resrcInst':
            data.value = decodeTlvResrc(path, id, value, offset, len);
            offset = offset + len;
            break;

        default:
            break;
    }

    data.id = id;
    data.type = type;
    data.offset = offset;

    return data;
}

function decodeTlvResrc(path, id, value, offset, len) {
    var oid = cutils.getPathIdKey(path).oid,
        rid = id,
        resrcDef = lwm2mId.getRdef(oid, rid),
        resrcType = resrcDef ? resrcDef.type : undefined,
        data;

    if (!resrcType) {
        // [TODO]
    }

    switch (resrcType) {
        case 'boolean':
            data = value.readUInt8(offset) ? true : false;
            break;

        case 'integer':
        case 'time':
            if (len == 1) {
                data = value.readInt8(offset);
            } else if (len == 2) {
                data = value.readInt16BE(offset);
            } else if (len <= 4) {
                data = value.readInt32BE(offset);
            } else if (len <= 8) {
                if (value[offset] & 0x80)
                    data = ((value.readInt32BE(offset) - 1)  * 0x100000000) + (value.readUInt32BE(offset + 4) - 0x100000000);
                else 
                    data = value.readInt32BE(offset) * 0x100000000 + value.readUInt32BE(offset + 4);
            }
            break;

        case 'float':
            data = value.readFloatBE(offset);
            break;

        case 'string':
            data = value.toString('utf8', offset, offset + len);
            break;

        default:
            data = value.toString('hex', offset, offset + len);
            break;
    }

    return data;
}

/*********************************************************
 * Json                                                  *
 *********************************************************/
function decodeJson(basePath, value) {
    var data = {},
        path = basePath,       // [TODO] bn is optional, so we need basePath to check type
        pathType = cutils.getPathDateType(path),
        oid = cutils.getPathIdKey(path).oid,
        rid;

    if (_.isBuffer(value) || _.isString(value))
        value = JSON.parse(value);

    if (!_.isPlainObject(value) || !value.e) 
        throw new TypeError('value should be an object.');

    switch (pathType) {
        case 'object':         // obj
            _.forEach(value.e, function (resrc) {
                var pathArray = cutils.getPathArray(resrc.n),          // [iid, rid[, riid]]
                    val;

                if (!_.isUndefined(resrc.v)) {
                    val = resrc.v;
                } else if (!_.isUndefined(resrc.sv)) {
                    val = resrc.sv;
                } else if (!_.isUndefined(resrc.bv)) {
                    val = resrc.bv;
                } else if (!_.isUndefined(resrc.ov)) {
                    val = resrc.ov;     // [TODO] objlnk
                }

                if (pathArray[0] === '')
                    pathArray = pathArray.slice(1);

                if (pathArray[pathArray.length - 1] === '')
                    pathArray = pathArray.slice(0, pathArray.length - 1);

                if (pathArray[0] && !_.has(data, pathArray[0]))
                    data[pathArray[0]] = {};

                rid = cutils.ridKey(oid, pathArray[1]);

                if (rid && !_.has(data, [pathArray[0], rid])) {
                    if (pathArray[2]) {
                        data[pathArray[0]][rid] = {};
                        data[pathArray[0]][rid][pathArray[2]] = val;
                    } else {
                        data[pathArray[0]][rid] = val;
                    }
                }
            });
            break;

        case 'instance':         // inst
            _.forEach(value.e, function (resrc) {
                var pathArray = cutils.getPathArray(resrc.n),          // [rid[, riid]]
                    val;

                if (!_.isUndefined(resrc.v)) {
                    val = resrc.v;
                } else if (!_.isUndefined(resrc.sv)) {
                    val = resrc.sv;
                } else if (!_.isUndefined(resrc.bv)) {
                    val = resrc.bv;
                } else if (!_.isUndefined(resrc.ov)) {
                    val = resrc.ov;     // [TODO] objlnk
                }

                if (pathArray[0] === '')
                    pathArray = pathArray.slice(1);

                if (pathArray[pathArray.length - 1] === '')
                    pathArray = pathArray.slice(0, pathArray.length - 1);

                rid = cutils.ridKey(oid, pathArray[0]);

                if (rid && !_.has(data, rid)) {
                    if (pathArray[1]) {
                        data[rid] = {};
                        data[rid][pathArray[1]] = val;
                    } else {
                        data[rid] = val;
                    }
                }
            });
            break;

        case 'resource':         // resrc
            _.forEach(value.e, function (resrc) {
                var riid = resrc.n,          // [[riid]]
                    isMultiResrc = false,
                    val;

                if (riid)
                    isMultiResrc = true;

                if (!_.isUndefined(resrc.v)) {
                    val = resrc.v;
                } else if (!_.isUndefined(resrc.sv)) {
                    val = resrc.sv;
                } else if (!_.isUndefined(resrc.bv)) {
                    val = resrc.bv;
                } else if (!_.isUndefined(resrc.ov)) {
                    val = resrc.ov;     // [TODO] objlnk
                }

                if (!isMultiResrc) {
                    data = val;
                } else if (isMultiResrc && !_.has(data, riid)) {
                    data[riid] = val;
                }
            });
            break;

        default:
            break;
     }
    
    return data;
}

/*********************************************************
 * Module Exports                                        *
 *********************************************************/
module.exports = decode;
