var expect = require('chai').expect,
    ObjectInstance = require('../lib/object_instance'); 

var objectInst = new ObjectInstance('temperature', 0, { ipsoOnly: false });

describe('Object Instance - Signature Check', function () {
    describe('#.init()', function () {
        it('should be a function', function () {
            expect(objectInst.init).to.be.a('function');
        });

        it('should throw TypeError if resrcs is not an object', function () {
            expect(function () { return objectInst.init(); }).to.throw(TypeError);
            expect(function () { return objectInst.init(undefined); }).to.throw(TypeError);
            expect(function () { return objectInst.init(null); }).to.throw(TypeError);
            expect(function () { return objectInst.init(NaN); }).to.throw(TypeError);
            expect(function () { return objectInst.init(10); }).to.throw(TypeError);
            expect(function () { return objectInst.init('xx'); }).to.throw(TypeError);
            expect(function () { return objectInst.init([]); }).to.throw(TypeError);
            expect(function () { return objectInst.init(true); }).to.throw(TypeError);
            expect(function () { return objectInst.init(new Date()); }).to.throw(TypeError);
            expect(function () { return objectInst.init(function () {}); }).to.throw(TypeError);

            expect(function () { return objectInst.init({}); }).not.to.throw(TypeError);
        });

        it('should throw TypeError if given setup is not a function', function () {
            expect(function () { return objectInst.init({ 5700: 30 }, 10); }).to.throw(TypeError);
            expect(function () { return objectInst.init({ 5700: 30 }, 'xx'); }).to.throw(TypeError);
            expect(function () { return objectInst.init({ 5700: 30 }, []); }).to.throw(TypeError);
            expect(function () { return objectInst.init({ 5700: 30 }, true); }).to.throw(TypeError);
            expect(function () { return objectInst.init({ 5700: 30 }, new Date()); }).to.throw(TypeError);

            expect(function () { return objectInst.init({ 5700: 30 }, function () {}); }).not.to.throw(TypeError);
        });

        it('should throw TypeError if iid is not a number when ipsoOnly is true', function () {
            objectInst.parent.ipsoOnly = true;
            expect(function () { return objectInst.init({ 9453: 30 }, function () {}); }).to.throw(TypeError);
        });
    });

    describe('#.has()', function () {
        it('should be a function', function () {
            expect(objectInst.has).to.be.a('function');
        });

        it('should throw TypeError if rid is not a string or a number', function () {
            expect(function () { return objectInst.has(); }).to.throw(TypeError);
            expect(function () { return objectInst.has(undefined); }).to.throw(TypeError);
            expect(function () { return objectInst.has(null); }).to.throw(TypeError);
            expect(function () { return objectInst.has(NaN); }).to.throw(TypeError);
            expect(function () { return objectInst.has([]); }).to.throw(TypeError);
            expect(function () { return objectInst.has({}); }).to.throw(TypeError);
            expect(function () { return objectInst.has(true); }).to.throw(TypeError);
            expect(function () { return objectInst.has(new Date()); }).to.throw(TypeError);
            expect(function () { return objectInst.has(function () {}); }).to.throw(TypeError);

            expect(function () { return objectInst.has(5700); }).not.to.throw(TypeError);
            expect(function () { return objectInst.has('temperature'); }).not.to.throw(TypeError);
        });
    });

    describe('#.get()', function () {
        it('should be a function', function () {
            expect(objectInst.get).to.be.a('function');
        });

        it('should throw TypeError if rid is not a string or a number', function () {
            expect(function () { return objectInst.get(); }).to.throw(TypeError);
            expect(function () { return objectInst.get(undefined); }).to.throw(TypeError);
            expect(function () { return objectInst.get(null); }).to.throw(TypeError);
            expect(function () { return objectInst.get(NaN); }).to.throw(TypeError);
            expect(function () { return objectInst.get([]); }).to.throw(TypeError);
            expect(function () { return objectInst.get({}); }).to.throw(TypeError);
            expect(function () { return objectInst.get(true); }).to.throw(TypeError);
            expect(function () { return objectInst.get(new Date()); }).to.throw(TypeError);
            expect(function () { return objectInst.get(function () {}); }).to.throw(TypeError);

            expect(function () { return objectInst.get(5700); }).not.to.throw(TypeError);
            expect(function () { return objectInst.get('temperature'); }).not.to.throw(TypeError);
        });
    });

    describe('#.set()', function () {
        it('should be a function', function () {
            expect(objectInst.set).to.be.a('function');
        });

        it('should throw TypeError if rid is not a string or a number', function () {
            expect(function () { return objectInst.set(undefined, 'xx'); }).to.throw(TypeError);
            expect(function () { return objectInst.set(null, 'xx'); }).to.throw(TypeError);
            expect(function () { return objectInst.set(NaN, 'xx'); }).to.throw(TypeError);
            expect(function () { return objectInst.set([], 'xx'); }).to.throw(TypeError);
            expect(function () { return objectInst.set({}, 'xx'); }).to.throw(TypeError);
            expect(function () { return objectInst.set(true, 'xx'); }).to.throw(TypeError);
            expect(function () { return objectInst.set(new Date(), 'xx'); }).to.throw(TypeError);
            expect(function () { return objectInst.set(function () {}, 'xx'); }).to.throw(TypeError);

            expect(function () { return objectInst.set(5700, 'xx'); }).not.to.throw(TypeError);
            expect(function () { return objectInst.set('temperature', 'xx'); }).not.to.throw(TypeError);
        });

        it('should throw TypeError if value is undefined or a function', function () {
            expect(function () { return objectInst.set(5700, undefined); }).to.throw(TypeError);
            expect(function () { return objectInst.set(5700, function () {}); }).to.throw(TypeError);

            expect(function () { return objectInst.set(5700, null); }).not.to.throw(TypeError);
            expect(function () { return objectInst.set(5700, NaN); }).not.to.throw(TypeError);
            expect(function () { return objectInst.set(5700, []); }).not.to.throw(TypeError);
            expect(function () { return objectInst.set(5700, {}); }).not.to.throw(TypeError);
            expect(function () { return objectInst.set(5700, true); }).not.to.throw(TypeError);
            expect(function () { return objectInst.set(5700, new Date()); }).not.to.throw(TypeError);
        });
    });

    describe('#.dump()', function () {
        it('should be a function', function () {
            expect(objectInst.dump).to.be.a('function');
        });

        it('should throw TypeError if given opt is not an object', function () {
            var cb = function () {};
            expect(function () { return objectInst.dump(10, cb); }).to.throw(TypeError);
            expect(function () { return objectInst.dump('xx', cb); }).to.throw(TypeError);
            expect(function () { return objectInst.dump([], cb); }).to.throw(TypeError);
            expect(function () { return objectInst.dump(true, cb); }).to.throw(TypeError);
            expect(function () { return objectInst.dump(new Date(), cb); }).to.throw(TypeError);

            expect(function () { return objectInst.dump(function () {}); }).not.to.throw(TypeError);
            expect(function () { return objectInst.dump({}, cb); }).not.to.throw(TypeError);
        });
    });

    describe('#.dumpSync()', function () {
        it('should be a function', function () {
            expect(objectInst.dumpSync).to.be.a('function');
        });
    });

    describe('#.read()', function () {
        it('should be a function', function () {
            expect(objectInst.read).to.be.a('function');
        });

        it('should throw TypeError if rid is not a string or a number', function () {
            expect(function () { return objectInst.read(); }).to.throw(TypeError);
            expect(function () { return objectInst.read(undefined); }).to.throw(TypeError);
            expect(function () { return objectInst.read(null); }).to.throw(TypeError);
            expect(function () { return objectInst.read(NaN); }).to.throw(TypeError);
            expect(function () { return objectInst.read([]); }).to.throw(TypeError);
            expect(function () { return objectInst.read({}); }).to.throw(TypeError);
            expect(function () { return objectInst.read(true); }).to.throw(TypeError);
            expect(function () { return objectInst.read(new Date()); }).to.throw(TypeError);
            expect(function () { return objectInst.read(function () {}); }).to.throw(TypeError);

            expect(function () { return objectInst.read(5700); }).not.to.throw(TypeError);
            expect(function () { return objectInst.read('temperature'); }).not.to.throw(TypeError);
        });

        it('should throw TypeError if given opt is not an object', function () {
            var cb = function () {};
            expect(function () { return objectInst.read(5700, 10, cb); }).to.throw(TypeError);
            expect(function () { return objectInst.read(5700, 'xx', cb); }).to.throw(TypeError);
            expect(function () { return objectInst.read(5700, [], cb); }).to.throw(TypeError);
            expect(function () { return objectInst.read(5700, true, cb); }).to.throw(TypeError);
            expect(function () { return objectInst.read(5700, new Date(), cb); }).to.throw(TypeError);

            expect(function () { return objectInst.read(5700, function () {}); }).not.to.throw(TypeError);
            expect(function () { return objectInst.read(5700, {}); }).not.to.throw(TypeError);
        });
    });

    describe('#.write()', function () {
        it('should be a function', function () {
            expect(objectInst.write).to.be.a('function');
        });

        it('should throw TypeError if rid is not a string or a number', function () {
            expect(function () { return objectInst.write(); }).to.throw(TypeError);
            expect(function () { return objectInst.write(undefined); }).to.throw(TypeError);
            expect(function () { return objectInst.write(null); }).to.throw(TypeError);
            expect(function () { return objectInst.write(NaN); }).to.throw(TypeError);
            expect(function () { return objectInst.write([]); }).to.throw(TypeError);
            expect(function () { return objectInst.write({}); }).to.throw(TypeError);
            expect(function () { return objectInst.write(true); }).to.throw(TypeError);
            expect(function () { return objectInst.write(new Date()); }).to.throw(TypeError);
            expect(function () { return objectInst.write(function () {}); }).to.throw(TypeError);

            expect(function () { return objectInst.write(5700, 30); }).not.to.throw(TypeError);
            expect(function () { return objectInst.write('temperature', 30); }).not.to.throw(TypeError);
        });

        it('should throw TypeError if value is undefined or a function', function () {
            expect(function () { return objectInst.write(5700, undefined); }).to.throw(TypeError);
            expect(function () { return objectInst.write(5700, function () {}); }).to.throw(TypeError);

            expect(function () { return objectInst.write(5700, null); }).not.to.throw(TypeError);
            expect(function () { return objectInst.write(5700, NaN); }).not.to.throw(TypeError);
            expect(function () { return objectInst.write(5700, []); }).not.to.throw(TypeError);
            expect(function () { return objectInst.write(5700, {}); }).not.to.throw(TypeError);
            expect(function () { return objectInst.write(5700, true); }).not.to.throw(TypeError);
            expect(function () { return objectInst.write(5700, new Date()); }).not.to.throw(TypeError);
        });

        it('should throw TypeError if given opt is not an object', function () {
            var cb = function () {};
            expect(function () { return objectInst.write(5700, 30, 10, cb); }).to.throw(TypeError);
            expect(function () { return objectInst.write(5700, 30, 'xx', cb); }).to.throw(TypeError);
            expect(function () { return objectInst.write(5700, 30, [], cb); }).to.throw(TypeError);
            expect(function () { return objectInst.write(5700, 30, true, cb); }).to.throw(TypeError);
            expect(function () { return objectInst.write(5700, 30, new Date(), cb); }).to.throw(TypeError);

            expect(function () { return objectInst.write(5700, 30, function () {}); }).not.to.throw(TypeError);
            expect(function () { return objectInst.write(5700, 30, {}); }).not.to.throw(TypeError);
        });
    });

    describe('#.exec()', function () {
        it('should be a function', function () {
            expect(objectInst.exec).to.be.a('function');
        });

        it('should throw TypeError if rid is not a string or a number', function () {
            expect(function () { return objectInst.exec(); }).to.throw(TypeError);
            expect(function () { return objectInst.exec(undefined); }).to.throw(TypeError);
            expect(function () { return objectInst.exec(null); }).to.throw(TypeError);
            expect(function () { return objectInst.exec(NaN); }).to.throw(TypeError);
            expect(function () { return objectInst.exec([]); }).to.throw(TypeError);
            expect(function () { return objectInst.exec({}); }).to.throw(TypeError);
            expect(function () { return objectInst.exec(true); }).to.throw(TypeError);
            expect(function () { return objectInst.exec(new Date()); }).to.throw(TypeError);
            expect(function () { return objectInst.exec(function () {}); }).to.throw(TypeError);

            expect(function () { return objectInst.exec(5700); }).not.to.throw(TypeError);
            expect(function () { return objectInst.exec('temperature'); }).not.to.throw(TypeError);
        });
    });

    describe('#.clear()', function () {
        it('should be a function', function () {
            expect(objectInst.clear).to.be.a('function');
        });
    });
});
