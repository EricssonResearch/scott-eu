var expect = require('chai').expect, 
    SmartObject = require('../index.js'); 

var smartObj = new SmartObject();

describe('Smart Object - Signature Check', function () {
    describe('#.init()', function () {
        it('should be a function', function () {
            expect(smartObj.init).to.be.a('function');
        });

        it('should throw TypeError if oid is not a string or a number', function () {
            expect(function () { return smartObj.init(); }).to.throw(TypeError);
            expect(function () { return smartObj.init(undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.init(null); }).to.throw(TypeError);
            expect(function () { return smartObj.init(NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.init([]); }).to.throw(TypeError);
            expect(function () { return smartObj.init({}); }).to.throw(TypeError);
            expect(function () { return smartObj.init(true); }).to.throw(TypeError);
            expect(function () { return smartObj.init(new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.init(function () {}); }).to.throw(TypeError);
        });

        it('should throw TypeError if iid is not a string or a number', function () {
            expect(function () { return smartObj.init(3302); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, null); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, []); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, {}); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, true); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, function () {}); }).to.throw(TypeError);
        });

        it('should throw TypeError if resrcs is not an object', function () {
            expect(function () { return smartObj.init(3302, 1, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, 2, null); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, 3, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, 4, 10); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, 5, 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, 6, []); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, 7, true); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, 8, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, 9, function () {}); }).to.throw(TypeError);

            expect(function () { return smartObj.init(3302, 0, {}); }).not.to.throw(TypeError);
        });

        it('should throw TypeError if given opt is not an object', function () {
            expect(function () { return smartObj.init(3302, 11, {}, 10); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, 12, {}, 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, 13, {}, []); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, 14, {}, true); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, 15, {}, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.init(3302, 17, {}, {}); }).to.throw(TypeError);

            expect(function () { return smartObj.init(3302, 16, {}, function () {}); }).not.to.throw(TypeError);
        });

        it('should throw TypeError if iid is not a number when ipsoOnly is true', function () {
            smartObj.ipsoOnly = true;
            expect(function () { return smartObj.init(3304, 'x', {}); }).to.throw(TypeError);
        });

        it('should throw Error if oid is not in an IPSO-defined when ipsoOnly is true', function () {
            smartObj.ipsoOnly = true;
            expect(function () { return smartObj.init(9453, 0, {}); }).to.throw(Error);
        });

        it('should throw Error if rid is not in an IPSO-defined when ipsoOnly is true', function () {
            smartObj.ipsoOnly = true;
            expect(function () { return smartObj.init(3304, 0, { 9999: 20 }); }).to.throw(Error);
        });
    });

    describe('#.create()', function () {
        it('should be a function', function () {
            expect(smartObj.create).to.be.a('function');
        });

        it('should throw TypeError if oid is not a string or a number', function () {
            expect(function () { return smartObj.create(); }).to.throw(TypeError);
            expect(function () { return smartObj.create(undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.create(null); }).to.throw(TypeError);
            expect(function () { return smartObj.create(NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.create([]); }).to.throw(TypeError);
            expect(function () { return smartObj.create({}); }).to.throw(TypeError);
            expect(function () { return smartObj.create(true); }).to.throw(TypeError);
            expect(function () { return smartObj.create(new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.create(function () {}); }).to.throw(TypeError);
        });

        it('should throw TypeError if iid is not a string or a number', function () {
            smartObj.ipsoOnly = false;

            expect(function () { return smartObj.create(3304); }).to.throw(TypeError);
            expect(function () { return smartObj.create(3304, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.create(3304, null); }).to.throw(TypeError);
            expect(function () { return smartObj.create(3304, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.create(3304, []); }).to.throw(TypeError);
            expect(function () { return smartObj.create(3304, {}); }).to.throw(TypeError);
            expect(function () { return smartObj.create(3304, true); }).to.throw(TypeError);
            expect(function () { return smartObj.create(3304, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.create(3304, function () {}); }).to.throw(TypeError);

            expect(function () { return smartObj.create(3304, 2); }).not.to.throw(TypeError);
            expect(function () { return smartObj.create(3304, 'b'); }).not.to.throw(TypeError);
        });

        it('should throw TypeError if iid is not a number when ipsoOnly is true', function () {
            expect(function () {
                smartObj.ipsoOnly = true;
                return smartObj.create(3304, 'x1');
            }).to.throw(TypeError);
        });

        it('should throw Error if oid is not in an IPSO-defined when ipsoOnly is true', function () {
            smartObj.ipsoOnly = true;
            expect(function () { return smartObj.create(9453, 0); }).to.throw(Error);
        });
    });

    describe('#.remove()', function () {
        it('should be a function', function () {
            expect(smartObj.remove).to.be.a('function');
        });

        it('should throw TypeError if oid is not a string or a number', function () {
            expect(function () { return smartObj.remove(); }).to.throw(TypeError);
            expect(function () { return smartObj.remove(undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.remove(null); }).to.throw(TypeError);
            expect(function () { return smartObj.remove(NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.remove([]); }).to.throw(TypeError);
            expect(function () { return smartObj.remove({}); }).to.throw(TypeError);
            expect(function () { return smartObj.remove(true); }).to.throw(TypeError);
            expect(function () { return smartObj.remove(new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.remove(function () {}); }).to.throw(TypeError);
        });

        it('should throw TypeError if iid is not a string or a number', function () {
            expect(function () { return smartObj.remove(3304); }).to.throw(TypeError);
            expect(function () { return smartObj.remove(3304, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.remove(3304, null); }).to.throw(TypeError);
            expect(function () { return smartObj.remove(3304, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.remove(3304, []); }).to.throw(TypeError);
            expect(function () { return smartObj.remove(3304, {}); }).to.throw(TypeError);
            expect(function () { return smartObj.remove(3304, true); }).to.throw(TypeError);
            expect(function () { return smartObj.remove(3304, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.remove(3304, function () {}); }).to.throw(TypeError);

            expect(function () { return smartObj.remove(3304, 2); }).not.to.throw(TypeError);
            expect(function () { return smartObj.remove(3304, 'b'); }).not.to.throw(TypeError);
        });
    });

    describe('#.objectList()', function () {
        it('should be a function', function () {
            expect(smartObj.objectList).to.be.a('function');
        });
    });

    describe('#.has()', function () {
        it('should be a function', function () {
            expect(smartObj.has).to.be.a('function');
        });

        it('should throw TypeError if input oid is not a number and not a string', function () {
            expect(function () { return smartObj.has(); }).to.throw(TypeError);
            expect(function () { return smartObj.has(undefined, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.has(null, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.has(NaN, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.has([], '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.has({}, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.has(true, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.has(new Date(), '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.has(function () {}, '1', 'a'); }).to.throw(TypeError);
        });

        it('should throw TypeError if input iid is not a number and not a string', function () {
            smartObj.init(3303, 0, {});
            expect(function () { return smartObj.has(3303, null, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.has(3303, NaN, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.has(3303, [], 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.has(3303, {}, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.has(3303, true, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.has(3303, new Date(), 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.has(3303, function () {}, 'a'); }).to.throw(TypeError);

        });

        it('should throw TypeError if input rid is not a number and not a string', function () {
            expect(function () { return smartObj.has(3303, 0, null); }).to.throw(TypeError);
            expect(function () { return smartObj.has(3303, 0, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.has(3303, 0, []); }).to.throw(TypeError);
            expect(function () { return smartObj.has(3303, 0, {}); }).to.throw(TypeError);
            expect(function () { return smartObj.has(3303, 0, true); }).to.throw(TypeError);
            expect(function () { return smartObj.has(3303, 0, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.has(3303, 0, function () {}); }).to.throw(TypeError);
        });
    });

    describe('#.findObject()', function () {
        it('should be a function', function () {
            expect(smartObj.findObject).to.be.a('function');
        });

        it('should throw TypeError if oid is not a string or a number', function () {
            expect(function () { return smartObj.findObject(); }).to.throw(TypeError);
            expect(function () { return smartObj.findObject(undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.findObject(null); }).to.throw(TypeError);
            expect(function () { return smartObj.findObject(NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.findObject([]); }).to.throw(TypeError);
            expect(function () { return smartObj.findObject({}); }).to.throw(TypeError);
            expect(function () { return smartObj.findObject(true); }).to.throw(TypeError);
            expect(function () { return smartObj.findObject(new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.findObject(function () {}); }).to.throw(TypeError);

            expect(function () { return smartObj.findObject(5700); }).not.to.throw(TypeError);
            expect(function () { return smartObj.findObject('temperature'); }).not.to.throw(TypeError);
        });
    });

    describe('#.findObjectInstance()', function () {
        it('should be a function', function () {
            expect(smartObj.findObjectInstance).to.be.a('function');
        });

        it('should throw TypeError if oid is not a string or a number', function () {
            expect(function () { return smartObj.findObjectInstance(); }).to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance(undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance(null); }).to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance(NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance([]); }).to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance({}); }).to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance(true); }).to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance(new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance(function () {}); }).to.throw(TypeError);
        });

        it('should throw TypeError if iid is not a string or a number', function () {
            expect(function () { return smartObj.findObjectInstance(5700); }).to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance(5700, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance(5700, null); }).to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance(5700, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance(5700, []); }).to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance(5700, {}); }).to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance(5700, true); }).to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance(5700, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance(5700, function () {}); }).to.throw(TypeError);

            expect(function () { return smartObj.findObjectInstance(5700, 0); }).not.to.throw(TypeError);
            expect(function () { return smartObj.findObjectInstance(5700, 'xx'); }).not.to.throw(TypeError);
        });
    });

    describe('#.isReadable()', function () {
        it('should be a function', function () {
            expect(smartObj.isReadable).to.be.a('function');
        });

        it('should throw TypeError if input oid is not a number and not a string', function () {
            expect(function () { return smartObj.isReadable(); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(undefined, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(null, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(NaN, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable([], '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable({}, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(true, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(new Date(), '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(function () {}, '1', 'a'); }).to.throw(TypeError);
        });

        it('should throw TypeError if input iid is not a number and not a string', function () {
            expect(function () { return smartObj.isReadable(3303); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(3303, undefined, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(3303, null, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(3303, NaN, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(3303, [], 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(3303, {}, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(3303, true, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(3303, new Date(), 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(3303, function () {}, 'a'); }).to.throw(TypeError);

        });

        it('should throw TypeError if input rid is not a number and not a string', function () {
            smartObj.init(3303, 1, {});
            expect(function () { return smartObj.isReadable(3303, 1); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(3303, 1, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(3303, 1, null); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(3303, 1, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(3303, 1, []); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(3303, 1, {}); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(3303, 1, true); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(3303, 1, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.isReadable(3303, 1, function () {}); }).to.throw(TypeError);
        });
    });

    describe('#.isWritable()', function () {
        it('should be a function', function () {
            expect(smartObj.isWritable).to.be.a('function');
        });

        it('should throw TypeError if input oid is not a number and not a string', function () {
            expect(function () { return smartObj.isWritable(); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(undefined, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(null, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(NaN, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable([], '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable({}, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(true, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(new Date(), '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(function () {}, '1', 'a'); }).to.throw(TypeError);
        });

        it('should throw TypeError if input iid is not a number and not a string', function () {
            expect(function () { return smartObj.isWritable(3303); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(3303, undefined, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(3303, null, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(3303, NaN, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(3303, [], 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(3303, {}, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(3303, true, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(3303, new Date(), 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(3303, function () {}, 'a'); }).to.throw(TypeError);

        });

        it('should throw TypeError if input rid is not a number and not a string', function () {
            expect(function () { return smartObj.isWritable(3303, 1); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(3303, 1, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(3303, 1, null); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(3303, 1, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(3303, 1, []); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(3303, 1, {}); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(3303, 1, true); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(3303, 1, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.isWritable(3303, 1, function () {}); }).to.throw(TypeError);
        });
    });
    
    describe('#.isExecutable()', function () {
        it('should be a function', function () {
            expect(smartObj.isExecutable).to.be.a('function');
        });

        it('should throw TypeError if input oid is not a number and not a string', function () {
            expect(function () { return smartObj.isExecutable(); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(undefined, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(null, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(NaN, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable([], '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable({}, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(true, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(new Date(), '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(function () {}, '1', 'a'); }).to.throw(TypeError);
        });

        it('should throw TypeError if input iid is not a number and not a string', function () {
            expect(function () { return smartObj.isExecutable(3303); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(3303, undefined, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(3303, null, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(3303, NaN, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(3303, [], 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(3303, {}, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(3303, true, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(3303, new Date(), 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(3303, function () {}, 'a'); }).to.throw(TypeError);

        });

        it('should throw TypeError if input rid is not a number and not a string', function () {
            expect(function () { return smartObj.isExecutable(3303, 1); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(3303, 1, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(3303, 1, null); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(3303, 1, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(3303, 1, []); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(3303, 1, {}); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(3303, 1, true); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(3303, 1, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.isExecutable(3303, 1, function () {}); }).to.throw(TypeError);
        });
    });

    describe('#.get()', function () {
        it('should be a function', function () {
            expect(smartObj.get).to.be.a('function');
        });

        it('should throw TypeError if input oid is not a number and not a string', function () {
            expect(function () { return smartObj.get(); }).to.throw(TypeError);
            expect(function () { return smartObj.get(undefined, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.get(null, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.get(NaN, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.get([], '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.get({}, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.get(true, '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.get(new Date(), '1', 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.get(function () {}, '1', 'a'); }).to.throw(TypeError);
        });

        it('should throw TypeError if input iid is not a number and not a string', function () {
            expect(function () { return smartObj.get(3303); }).to.throw(TypeError);
            expect(function () { return smartObj.get(3303, undefined, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.get(3303, null, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.get(3303, NaN, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.get(3303, [], 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.get(3303, {}, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.get(3303, true, 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.get(3303, new Date(), 'a'); }).to.throw(TypeError);
            expect(function () { return smartObj.get(3303, function () {}, 'a'); }).to.throw(TypeError);

        });

        it('should throw TypeError if input rid is not a number and not a string', function () {
            expect(function () { return smartObj.get(3303, 1); }).to.throw(TypeError);
            expect(function () { return smartObj.get(3303, 1, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.get(3303, 1, null); }).to.throw(TypeError);
            expect(function () { return smartObj.get(3303, 1, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.get(3303, 1, []); }).to.throw(TypeError);
            expect(function () { return smartObj.get(3303, 1, {}); }).to.throw(TypeError);
            expect(function () { return smartObj.get(3303, 1, true); }).to.throw(TypeError);
            expect(function () { return smartObj.get(3303, 1, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.get(3303, 1, function () {}); }).to.throw(TypeError);
        });
    });

    describe('#.set()', function () {
        it('should be a function', function () {
            expect(smartObj.set).to.be.a('function');
        });

        it('should throw TypeError if input oid is not a number and not a string', function () {
            expect(function () { return smartObj.set(); }).to.throw(TypeError);
            expect(function () { return smartObj.set(undefined, '1', 'a', 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(null, '1', 'a', 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(NaN, '1', 'a', 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set([], '1', 'a', 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set({}, '1', 'a', 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(true, '1', 'a', 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(new Date(), '1', 'a', 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(function () {}, '1', 'a', 'xx'); }).to.throw(TypeError);
        });

        it('should throw TypeError if input iid is not a number and not a string', function () {
            expect(function () { return smartObj.set(3303); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, undefined, 'a', 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, null, 'a', 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, NaN, 'a', 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, [], 'a', 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, {}, 'a', 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, true, 'a', 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, new Date(), 'a', 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, function () {}, 'a', 'xx'); }).to.throw(TypeError);

        });

        it('should throw TypeError if input rid is not a number and not a string', function () {
            smartObj.init(3303, 2, {});
            expect(function () { return smartObj.set(3303, 2); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, 2, undefined, 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, 2, null, 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, 2, NaN, 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, 2, [], 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, 2, {}, 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, 2, true, 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, 2, new Date(), 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, 2, function () {}, 'xx'); }).to.throw(TypeError);
        });

        it('should throw TypeError if value is undefined or a function', function () {
            expect(function () { return smartObj.set(3303, 2, 5700, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.set(3303, 2, 5700, function () {}); }).to.throw(TypeError);

            expect(function () { return smartObj.set(3303, 2, 5700, null); }).not.to.throw(TypeError);
            expect(function () { return smartObj.set(3303, 2, 5700, NaN); }).not.to.throw(TypeError);
            expect(function () { return smartObj.set(3303, 2, 5700, []); }).not.to.throw(TypeError);
            expect(function () { return smartObj.set(3303, 2, 5700, {}); }).not.to.throw(TypeError);
            expect(function () { return smartObj.set(3303, 2, 5700, true); }).not.to.throw(TypeError);
            expect(function () { return smartObj.set(3303, 2, 5700, new Date()); }).not.to.throw(TypeError);
        });
    });

    describe('#.dumpSync()', function () {
        it('should be a function', function () {
            expect(smartObj.dumpSync).to.be.a('function');
        });

        it('should throw TypeError if oid is not a string or a number', function () {
            expect(function () { return smartObj.dumpSync(undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.dumpSync(null); }).to.throw(TypeError);
            expect(function () { return smartObj.dumpSync(NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.dumpSync([]); }).to.throw(TypeError);
            expect(function () { return smartObj.dumpSync({}); }).to.throw(TypeError);
            expect(function () { return smartObj.dumpSync(true); }).to.throw(TypeError);
            expect(function () { return smartObj.dumpSync(new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.dumpSync(function () {}); }).to.throw(TypeError);

            expect(function () { return smartObj.dumpSync(3303); }).not.to.throw(TypeError);
            expect(function () { return smartObj.dumpSync('temperature'); }).not.to.throw(TypeError);
        });

        it('should throw TypeError if iid is not a string or a number', function () {
            expect(function () { return smartObj.dumpSync(3303, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.dumpSync(3303, null); }).to.throw(TypeError);
            expect(function () { return smartObj.dumpSync(3303, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.dumpSync(3303, []); }).to.throw(TypeError);
            expect(function () { return smartObj.dumpSync(3303, {}); }).to.throw(TypeError);
            expect(function () { return smartObj.dumpSync(3303, true); }).to.throw(TypeError);
            expect(function () { return smartObj.dumpSync(3303, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.dumpSync(3303, function () {}); }).to.throw(TypeError);

            expect(function () { return smartObj.dumpSync(3303, 0); }).not.to.throw(TypeError);
            expect(function () { return smartObj.dumpSync(3303, 'xx'); }).not.to.throw(TypeError);
        });
    });

    describe('#.dump()', function () {
        it('should be a function', function () {
            expect(smartObj.dump).to.be.a('function');
        });

        it('should throw TypeError if callback is not a function', function () {
            expect(function () { return smartObj.dump(); }).to.throw(Error);
            expect(function () { return smartObj.dump(undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(null); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(10); }).to.throw(TypeError);
            expect(function () { return smartObj.dump('xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.dump([]); }).to.throw(TypeError);
            expect(function () { return smartObj.dump({}); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(true); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(new Date()); }).to.throw(TypeError);

            expect(function () { return smartObj.dump(function () {}); }).not.to.throw(TypeError);
        });

        it('should throw TypeError if oid is not a string or a number', function () {
            expect(function () { return smartObj.dump(undefined, function () {}); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(null, function () {}); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(NaN, function () {}); }).to.throw(TypeError);
            expect(function () { return smartObj.dump([], function () {}); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(true, function () {}); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(new Date(), function () {}); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(function () {}, function () {}); }).to.throw(TypeError);

            expect(function () { return smartObj.dump({}, function () {}); }).not.to.throw(TypeError);          // opt
            expect(function () { return smartObj.dump(3303, {}, function () {}); }).not.to.throw(TypeError);    // opt
        });

        it('should throw TypeError if iid is not a string or a number', function () {
            expect(function () { return smartObj.dump(3303, undefined, function () {}); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(3303, null, function () {}); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(3303, NaN, function () {}); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(3303, [], function () {}); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(3303, true, function () {}); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(3303, new Date(), function () {}); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(3303, function () {}, function () {}); }).to.throw(TypeError);

            expect(function () { return smartObj.dump(3303, {}, function () {}); }).not.to.throw(TypeError);    // opt
            expect(function () { return smartObj.dump(3303, 0, {}, function () {}); }).not.to.throw(TypeError); // opt
        });

        it('should throw TypeError if given opt is not an object', function () {
            expect(function () { return smartObj.dump(3303, 2, function () {}, 10); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(3303, 2, function () {}, 'xx'); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(3303, 2, function () {}, []); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(3303, 2, function () {}, true); }).to.throw(TypeError);
            expect(function () { return smartObj.dump(3303, 2, function () {}, new Date()); }).to.throw(TypeError);

            expect(function () { return smartObj.dump(3303, 2, function () {}, function () {}); }).not.to.throw(TypeError);
            expect(function () { return smartObj.dump(3303, 2, {}, function () {}); }).not.to.throw(TypeError);
        });
    });

    describe('#.read()', function () {
        it('should be a function', function () {
            expect(smartObj.read).to.be.a('function');
        });

        it('should throw TypeError if oid is not a string or a number', function () {
            expect(function () { return smartObj.read(); }).to.throw(TypeError);
            expect(function () { return smartObj.read(undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.read(null); }).to.throw(TypeError);
            expect(function () { return smartObj.read(NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.read([]); }).to.throw(TypeError);
            expect(function () { return smartObj.read({}); }).to.throw(TypeError);
            expect(function () { return smartObj.read(true); }).to.throw(TypeError);
            expect(function () { return smartObj.read(new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.read(function () {}); }).to.throw(TypeError);
        });

        it('should throw TypeError if iid is not a string or a number', function () {
            expect(function () { return smartObj.read(3303); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, null); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, []); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, {}); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, true); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, function () {}); }).to.throw(TypeError);
        });

        it('should throw TypeError if rid is not a string or a number', function () {
            expect(function () { return smartObj.read(3303, 2); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, 2, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, 2, null); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, 2, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, 2, []); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, 2, {}); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, 2, true); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, 2, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, 2, function () {}); }).to.throw(TypeError);

            expect(function () { return smartObj.read(3303, 2, 5700); }).not.to.throw(TypeError);
            expect(function () { return smartObj.read(3303, 2, 'temperature'); }).not.to.throw(TypeError);
        });

        it('should throw TypeError if given opt is not an object', function () {
            var cb = function () {};
            expect(function () { return smartObj.read(3303, 2, 5700, 10, cb); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, 2, 5700, 'xx', cb); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, 2, 5700, [], cb); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, 2, 5700, true, cb); }).to.throw(TypeError);
            expect(function () { return smartObj.read(3303, 2, 5700, new Date(), cb); }).to.throw(TypeError);

            expect(function () { return smartObj.read(3303, 2, 5700, function () {}, cb); }).not.to.throw(TypeError);
            expect(function () { return smartObj.read(3303, 2, 5700, {}); }).not.to.throw(TypeError);
        });
    });

    describe('#.write()', function () {
        it('should be a function', function () {
            expect(smartObj.write).to.be.a('function');
        });

        it('should throw TypeError if oid is not a string or a number', function () {
            expect(function () { return smartObj.write(); }).to.throw(TypeError);
            expect(function () { return smartObj.write(undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.write(null); }).to.throw(TypeError);
            expect(function () { return smartObj.write(NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.write([]); }).to.throw(TypeError);
            expect(function () { return smartObj.write({}); }).to.throw(TypeError);
            expect(function () { return smartObj.write(true); }).to.throw(TypeError);
            expect(function () { return smartObj.write(new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.write(function () {}); }).to.throw(TypeError);
        });

        it('should throw TypeError if iid is not a string or a number', function () {
            expect(function () { return smartObj.write(5700); }).to.throw(TypeError);
            expect(function () { return smartObj.write(5700, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.write(5700, null); }).to.throw(TypeError);
            expect(function () { return smartObj.write(5700, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.write(5700, []); }).to.throw(TypeError);
            expect(function () { return smartObj.write(5700, {}); }).to.throw(TypeError);
            expect(function () { return smartObj.write(5700, true); }).to.throw(TypeError);
            expect(function () { return smartObj.write(5700, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.write(5700, function () {}); }).to.throw(TypeError);
        });

        it('should throw TypeError if rid is not a string or a number', function () {
            expect(function () { return smartObj.write(); }).to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, null); }).to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, []); }).to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, {}); }).to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, true); }).to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, function () {}); }).to.throw(TypeError);

            expect(function () { return smartObj.write(3303, 2, 5700, 30); }).not.to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, 'temperature', 30); }).not.to.throw(TypeError);
        });

        it('should throw TypeError if value is undefined or a function', function () {
            expect(function () { return smartObj.write(3303, 2, 5700, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, 5700, function () {}); }).to.throw(TypeError);

            expect(function () { return smartObj.write(3303, 2, 5700, null); }).not.to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, 5700, NaN); }).not.to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, 5700, []); }).not.to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, 5700, {}); }).not.to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, 5700, true); }).not.to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, 5700, new Date()); }).not.to.throw(TypeError);
        });

        it('should throw TypeError if given opt is not an object', function () {
            var cb = function () {};
            expect(function () { return smartObj.write(3303, 2, 5700, 30, 10, cb); }).to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, 5700, 30, 'xx', cb); }).to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, 5700, 30, [], cb); }).to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, 5700, 30, true, cb); }).to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, 5700, 30, new Date(), cb); }).to.throw(TypeError);

            expect(function () { return smartObj.write(3303, 2, 5700, 30, function () {}, cb); }).not.to.throw(TypeError);
            expect(function () { return smartObj.write(3303, 2, 5700, 30, {}); }).not.to.throw(TypeError);
        });
    });

    describe('#.exec()', function () {
        it('should be a function', function () {
            expect(smartObj.exec).to.be.a('function');
        });

        it('should throw TypeError if oid is not a string or a number', function () {
            expect(function () { return smartObj.exec(); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(null); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.exec([]); }).to.throw(TypeError);
            expect(function () { return smartObj.exec({}); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(true); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(function () {}); }).to.throw(TypeError);
        });

        it('should throw TypeError if iid is not a string or a number', function () {
            expect(function () { return smartObj.exec(5700); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(5700, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(5700, null); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(5700, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(5700, []); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(5700, {}); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(5700, true); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(5700, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(5700, function () {}); }).to.throw(TypeError);
        });

        it('should throw TypeError if rid is not a string or a number', function () {
            expect(function () { return smartObj.exec(); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(3303, 2, undefined); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(3303, 2, null); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(3303, 2, NaN); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(3303, 2, []); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(3303, 2, {}); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(3303, 2, true); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(3303, 2, new Date()); }).to.throw(TypeError);
            expect(function () { return smartObj.exec(3303, 2, function () {}); }).to.throw(TypeError);

            expect(function () { return smartObj.exec(3303, 2, 5700, 30); }).not.to.throw(TypeError);
            expect(function () { return smartObj.exec(3303, 2, 'temperature', 30); }).not.to.throw(TypeError);
        });
    });
});
