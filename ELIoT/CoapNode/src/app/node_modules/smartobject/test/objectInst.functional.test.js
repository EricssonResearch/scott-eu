var util = require('util'),
    _ = require('busyman'),
    expect = require('chai').expect,
    ObjectInstance = require('../lib/object_instance'); 

var objectInst = new ObjectInstance('temperature', 0, { ipsoOnly: false }),
    objectInst2 = new ObjectInstance('temperature', 0, { ipsoOnly: false }),
    objectInstPwr = new ObjectInstance('pwrMea', 0, { ipsoOnly: false });

describe('Object Instance - Functional Check', function () {
    describe('#.init()', function () {
        it('should add a Resource', function () {
            objectInst.init({ 5700: 30 });
            expect(objectInst.sensorValue).to.be.eql(30);
        });

        it('should clear all the Resources', function () {
            objectInst.init({ 5701: 'c' });
            expect(objectInst.sensorValue).to.be.eql(undefined);
            expect(objectInst.units).to.be.eql('c');
        });

        it('should return id and add a Resource when rsc include function', function () {
            objectInst.init({ 
                5700: {
                    read: function (cb) {
                        cb(null, 25);
                    }    
                }
            });

            expect(objectInst.sensorValue._isCb).to.be.true;
        });
    });

    describe('#.has()', function () {
        it('should return true if target is exist', function () {
            expect(objectInst.has(5700)).to.be.true;
            expect(objectInst.has('5700')).to.be.true;
            expect(objectInst.has('sensorValue')).to.be.true;
        });

        it('should return false if target is not exist', function () {
            expect(objectInst.has(5702)).to.be.false;
            expect(objectInst.has(5703)).to.be.false;
        });
    });

    describe('#.get()', function () {
        it('should return rsc if target is exist', function () {
            expect(typeof objectInst.get(5700).read).to.be.eql('function');
            expect(typeof objectInst.get('5700').read).to.be.eql('function');
            expect(typeof objectInst.get('sensorValue').read).to.be.eql('function');
        });

        it('should return undefined if target is not exist', function () {
            expect(objectInst.get(5702)).to.be.eql(undefined);
            expect(objectInst.get(5703)).to.be.eql(undefined);
        });
    });

    describe('#.set()', function () {
        it('should return true and add a Resource', function () {
            expect(objectInst.set(5701, 'f')).to.be.eql(objectInst);
            expect(objectInst.get(5701)).to.be.eql('f');
        });
    });

    describe('#.dumpSync()', function () {
        it('should return object instance', function () {
            var obj = {
                sensorValue: {
                    read: '_read_'
                },
                units: 'f'
            };

            expect(objectInst.dumpSync()).to.be.eql(obj);
        });
    });

    describe('#.dump()', function () {
        it('should get whole object instance', function (done) {
            var obj = { 
                sensorValue: 20,
                units: 'c',
                5702: '_unreadable_',
                5703: 'x',
                5704: '_exec_'
            };

            objectInst.init({ 
                5700: 20,
                5701: {
                    read: function (cb) {
                        cb(null, 'c');
                    }    
                },
                5702: {
                    write: function (val, cb) {
                        cb(null, val);
                    }    
                },
                5703: {
                    read: function (cb) {
                        cb(null, 'x');
                    },
                    write: function (val, cb) {
                        cb(null, val);
                    }    
                },
                5704: {
                    exec: function (val, cb) {
                        if (val === 20)
                            cb(null, true);
                        else 
                            cb(null, false);
                    } 
                }
            });

            objectInst.dump(function (err, result) {
                if (_.isEqual(result, obj)) {
                    done();
                }
            });
        });

        it('should get whole object instance whit restrict is true', function (done) {
            var obj = { 
                instActivePwr: 20,
                minMeaActivePwr: 'c',
                activePwrCal: '_unreadable_',
                currCal: 'x',
                resetCumulEnergy: '_exec_'
            };

            objectInstPwr.init({ 
                5800: 20,
                5801: 'c',
                5806: 'd',
                5821: 'x',
                5822: {
                    exec: function (val, cb) {
                        if (val === 20)
                            cb(null, true);
                        else 
                            cb(null, false);
                    } 
                }
            });

            objectInstPwr.dump({ restrict: true }, function (err, result) {
                if (_.isEqual(result, obj)) {
                    done();
                }
            });
        });

        it('should return error', function (done) {
            objectInst2.sensorValue = function () {};
            objectInst2.units = function () {};

            objectInst2.dump(function (err, result) {
                if (err) {
                    done();
                }
            });
        });
    });

    describe('#.read()', function () {
        it('should pass the read value when Resources is a primitive', function (done) {
            objectInst.read(5700, function (err, result) {
                if (result === 20) 
                    done();
            });
        });

        it('should pass the read value when Resources is readable', function (done) {
            objectInst.read(5701, function (err, result) {
                if (result === 'c') 
                    done();
            });
        });

        it('should pass the read value when Resources is readable and writable', function (done) {
            objectInst.read(5703, function (err, result) {
                if (result === 'x') 
                    done();
            });
        });

        it('should pass the read value when restrict is true', function (done) {
            objectInstPwr.read(5800, { restrict: true }, function (err, result) {
                if (result === 20) 
                    done();
            });
        });

        it('should pass the read value when restrict is true', function (done) {
            objectInstPwr.read(5801, { restrict: true }, function (err, result) {
                if (result === 'c') 
                    done();
            });
        });

        it('should pass the read value when restrict is true', function (done) {
            objectInstPwr.read(5821, function (err, result) {
                if (result === 'x') 
                    done();
            }, { restrict: true });
        });

        it('should pass _unreadable_ when restrict is true', function (done) {
            objectInstPwr.read(5806, { restrict: true }, function (err, result) {
                if (err && result === '_unreadable_') 
                    done();
            });
        });

        it('should pass _unreadable_ when Resources is writable', function (done) {
            objectInst.read(5702, function (err, result) {
                if (result === '_unreadable_')
                    done();
            });
        });

        it('should pass _exec_ when Resources is excutable ', function (done) {
            objectInst.read(5704, function (err, result) {
                if (result === '_exec_')
                    done();
            });
        });

        it('should pass not found error', function (done) {
            objectInst.read(5706, function (err, result) {
                if (err && result === '_notfound_')
                    done();
            });
        });
    });

    describe('#.write()', function () {
        it('should write Resource and pass the write value when Resources is a primitive', function (done) {
            objectInst.write(5700, 22, function (err, result) {
                if ((result === 22) && (objectInst.get(5700) === 22))
                    done();
            });
        });

        it('should and pass the write value when Resources is writable', function (done) {
            objectInst.write(5702, 24, function (err, result) {
                if (result === 24)
                    done();
            });
        });

        it('should pass the write value when Resources is readable and writable', function (done) {
            objectInst.write(5703, 26, function (err, result) {
                if (result === 26)
                    done();
            });
        });

        it('should pass the write value when restrict is true', function (done) {
            objectInstPwr.write(5806, 'f', { restrict: true }, function (err, result) {
                if (result === 'f' && (objectInstPwr.get(5806) === 'f')) 
                    done();
            });
        });

        it('should pass the write value when restrict is true', function (done) {
            objectInstPwr.write(5821, 'y', { restrict: true }, function (err, result) {
                if (result === 'y' && (objectInstPwr.get(5821) === 'y')) 
                    done();
            });
        });

        it('should pass the write value when restrict is true', function (done) {
            objectInstPwr.write(5800, 30, { restrict: true }, function (err, result) {
                if (err && result === '_unwritable_' && (objectInstPwr.get(5800) === 20)) 
                    done();
            });
        });

        it('should pass _unwritable_ when restrict is true', function (done) {
            objectInstPwr.write(5801, 'x', { restrict: true }, function (err, result) {
                if (err && result === '_unwritable_' && (objectInstPwr.get(5801) === 'c')) 
                    done();
            });
        });

        it('should pass _unwritable_ when Resources is readable', function (done) {
            objectInst.write(5701, 20, function (err, result) {
                if (result === '_unwritable_')
                    done();
            });
        });

        it('should pass _exec_ when Resources is excutable', function (done) {
            objectInst.write(5704, 20, function (err, result) {
                if (result === '_exec_')
                    done();
            });
        });

        it('should pass not found error', function (done) {
            objectInst.write(5706, 'C', function (err, result) {
                if (err && result === '_notfound_')
                    done();
            });
        });
    });

    describe('#.exec()', function () {
        it('should exec Resource and pass true through its second argument', function (done) {
            objectInst.exec(5704, [ 20 ], function (err, result) {
                if (result === true)
                    done();
            });
        });

        it('should pass _unexecutable_ when Resources is a primitive', function (done) {
            objectInst.exec(5700, [ 20 ], function (err, result) {
                if (result === '_unexecutable_')
                    done();
            });
        });

        it('should pass _unexecutable_ when Resources is readable', function (done) {
            objectInst.exec(5701, [ 20 ], function (err, result) {
                if (result === '_unexecutable_')
                    done();
            });
        });

        it('should pass _unexecutable_ when Resources is writable', function (done) {
            objectInst.exec(5702, [ 20 ], function (err, result) {
                if (result === '_unexecutable_')
                    done();
            });
        });

        it('should pass not argus error', function (done) {
            objectInst.exec(5704, '20', function (err, result) {
                if (err)
                    done();
            });
        });

        it('should pass not found error', function (done) {
            objectInst.exec(5706, [ 20 ], function (err, result) {
                if (err && result === '_notfound_')
                    done();
            });
        });
    });

    describe('#.clear()', function () {
        it('should return object instance and remove all Resources', function () {
            expect(objectInst.clear()).to.be.eql(new ObjectInstance('temperature', 0));
            expect(objectInst.get('sensorValue')).to.be.eql(undefined);
            expect(objectInst.get('units')).to.be.eql(undefined);
        });
    });
});
