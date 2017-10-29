var _ = require('busyman'),
    expect = require('chai').expect, 
    SmartObject = require('../index.js'); 

var smartObj = new SmartObject();

describe('Smart Object - Functional Check', function () {
    describe('#.init()', function () {
        it('should add a Resource', function () {
            smartObj.init(3303, 0, { 5700: 30 });
            expect(smartObj.temperature[0].sensorValue).to.be.eql(30);
        });

        it('should clear all the Resources', function () {
            smartObj.init(3303, 0, { 5701: 'c' });
            expect(smartObj.temperature[0].sensorValue).to.be.eql(undefined);
            expect(smartObj.temperature[0].units).to.be.eql('c');
        });

        it('should return id and add a Resource when rsc include function', function () {
            smartObj.init(3303, 0, { 
                5700: {
                    read: function (cb) {
                        cb(null, 25);
                    }    
                }
            });

            expect(typeof smartObj.temperature[0].sensorValue.read).to.be.eql('function');
            expect(smartObj.temperature[0].sensorValue._isCb).to.be.true;
        });

        it('should add a Multi-Resource', function () {
            smartObj.init('multiResource', 0, {
                5700: {
                    0: 'x',
                    1: 'y',
                    2: 'z'
                }
            });

            expect(smartObj.multiResource[0][5700][0]).to.be.eql('x');
            expect(smartObj.multiResource[0][5700]._isCb).to.be.false;
        });
    });

    describe('#.create()', function () {
        it('should create an Object Instance', function () {
            smartObj.create(3303, 1);
            expect(smartObj.temperature[1].oid).to.be.eql('temperature');
            expect(smartObj.temperature[1].iid).to.be.eql(1);
        });
    });

    describe('#.remove()', function () {
        it('should remove an Object Instance', function () {
            smartObj.init(3303, 2, { 
                5700: 20
            });
            expect(smartObj.temperature[2].sensorValue).to.be.eql(20);

            smartObj.remove(3303, 2);
            expect(smartObj.temperature[2]).to.be.eql(undefined);
            expect(smartObj.findObjectInstance(3303, 2)).to.be.eql(undefined);
        });
    });
    
    describe('#.objectList()', function () {
        it('should return objectList', function () {
            expect(smartObj.objectList()).to.be.eql([{ oid: 3303, iid: [ 0, 1 ]}, { oid: 'multiResource', iid: [0] }]);
        });
    });

    describe('#.has()', function () {
        it('should return true if target is exist', function () {
            expect(smartObj.has(3303, 0, 5700)).to.be.true;
            expect(smartObj.has('3303', '0', '5700')).to.be.true;
            expect(smartObj.has('temperature', 0, 'sensorValue')).to.be.true;
        });

        it('should return false if target is not exist', function () {
            expect(smartObj.has(3303, 0, 5702)).to.be.false;
            expect(smartObj.has(3303, 0, 5703)).to.be.false;
            expect(smartObj.has(3304, 0, 5700)).to.be.false;
            expect(smartObj.has(3304, 0, 5701)).to.be.false;
        });
    });

    describe('#.findObject()', function () {
        it('should return object if target is exist', function () {
            expect(smartObj.findObject(3303)).to.be.eql(smartObj.temperature);
            expect(smartObj.findObject('3303')).to.be.eql(smartObj.temperature);
            expect(smartObj.findObject('temperature')).to.be.eql(smartObj.temperature);
        });

        it('should return undefined if target is not exist', function () {
            expect(smartObj.findObject(3304)).to.be.eql(undefined);
            expect(smartObj.findObject(3305)).to.be.eql(undefined);
            expect(smartObj.findObject('3304')).to.be.eql(undefined);
            expect(smartObj.findObject('temper')).to.be.eql(undefined);
        });
    });

    describe('#.findObjectInstance()', function () {
        it('should return object instance if target is exist', function () {
            expect(smartObj.findObjectInstance(3303, 0)).to.be.eql(smartObj.temperature[0]);
            expect(smartObj.findObjectInstance(3303, 1)).to.be.eql(smartObj.temperature[1]);
            expect(smartObj.findObjectInstance('3303', '0')).to.be.eql(smartObj.temperature[0]);
            expect(smartObj.findObjectInstance('temperature', 0)).to.be.eql(smartObj.temperature[0]);
        });

        it('should return undefined if target is not exist', function () {
            expect(smartObj.findObjectInstance(3304, 0)).to.be.eql(undefined);
            expect(smartObj.findObjectInstance(3305, 0)).to.be.eql(undefined);
            expect(smartObj.findObjectInstance('3304', '1')).to.be.eql(undefined);
            expect(smartObj.findObjectInstance('temper', '2')).to.be.eql(undefined);
        });
    });

    describe('#.get()', function () {
        it('should return rsc if target is exist', function () {
            expect(typeof smartObj.get(3303, 0, 5700).read).to.be.eql('function');
            expect(typeof smartObj.get('3303', '0', '5700').read).to.be.eql('function');
            expect(typeof smartObj.get('temperature', 0, 'sensorValue').read).to.be.eql('function');
        });

        it('should return undefined if target is not exist', function () {
            expect(smartObj.get(3303, 0, 5702)).to.be.eql(undefined);
            expect(smartObj.get(3303, 0, 5703)).to.be.eql(undefined);
        });
    });

    describe('#.set()', function () {
        it('should return true and add a Resource', function () {
            expect(smartObj.set(3303, 1, 5701, 'f')).to.be.true;
            expect(smartObj.get(3303, 1, 5701)).to.be.eql('f');
        });

        it('should return false if target is not exist', function () {
            expect(smartObj.set(3303, 2, 5702, 'xx')).to.be.false;
            expect(smartObj.set(3304, 0, 5703, 'xx')).to.be.false;
        });
    });

    describe('#.dumpSync()', function () {
        it('should return whole smartObj', function () {
            var obj = {
                temperature: {
                    0: {
                        sensorValue: {
                            read: '_read_'
                        }
                    }, 
                    1: {
                        units: 'f'
                    }
                },
                multiResource: {
                    0: {
                        5700: {
                            0: 'x',
                            1: 'y',
                            2: 'z'
                        }
                    }
                }
            };

            expect(smartObj.dumpSync()).to.be.eql(obj);
        });

        it('should return object', function () {
            var obj = {
                0: {
                    sensorValue: {
                        read: '_read_'
                    }
                }, 
                1: {
                    units: 'f'
                }
            };

            expect(smartObj.dumpSync(3303)).to.be.eql(obj);
        });

        it('should return object instance', function () {
            var obj = {
                sensorValue: {
                    read: '_read_'
                }
            };

            expect(smartObj.dumpSync(3303, 0)).to.be.eql(obj);
        });
    });

    describe('#.dump()', function () {
        it('should get whole smartObj', function (done) {
            var obj = { 
                temperature: {
                    0: {
                        sensorValue: 20,
                        units: 'c',
                        5702: '_unreadable_',
                        5703: 'x',
                        5704: '_exec_'
                    },
                    1: {
                        units: 'f'
                    }
                },
                multiResource: {
                    0: {
                        5700: {
                            0: 'x',
                            1: 'y',
                            2: 'z'
                        }
                    }
                }
            };

            smartObj.init(3303, 0, { 
                sensorValue: 20,
                units: {
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

            smartObj.dump(function (err, result) {
                if (_.isEqual(result, obj)) 
                    done();
            });
        });

        it('should get an object', function (done) {
            var obj = { 
                0: {
                    sensorValue: 20,
                    units: 'c',
                    5702: '_unreadable_',
                    5703: 'x',
                    5704: '_exec_'
                },
                1: {
                    units: 'f'
                }
            };

            smartObj.dump(3303, function (err, result) {
                if (_.isEqual(result, obj)) 
                    done();
            });
        });

        it('should get an object instance', function (done) {
            var obj = { 
                sensorValue: 20,
                units: 'c',
                5702: '_unreadable_',
                5703: 'x',
                5704: '_exec_'
            };

            smartObj.dump(3303, 0, function (err, result) {
                if (_.isEqual(result, obj)) 
                    done();
            });
        });

        it('should get an object instance whit restrict is true', function (done) {
            var obj = { 
                instActivePwr: 20,
                minMeaActivePwr: 'c',
                activePwrCal: '_unreadable_',
                currCal: 'x',
                resetCumulEnergy: '_exec_'
            };

            smartObj.init(3305, 0, { 
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

            smartObj.dump(3305, 0, { restrict: true }, function (err, result) {
                if (_.isEqual(result, obj)) 
                    done();
            });
        });

        it('should return error', function (done) {
            smartObj.init(3200, 0, {});
            smartObj.init(3200, 1, {});
            smartObj.dIn[0].dInState = function() {};
            smartObj.dIn[0].counter = function() {};
            smartObj.dIn[1].dInState = function() {};
            smartObj.dIn[1].counter = function() {};

            smartObj.dump(3200, function (err, result) {
                if (err) 
                    done();
            });
        });

        it('should return error', function (done) {
            smartObj.init(3201, 0, {});
            smartObj.init(3201, 1, {});
            smartObj.dOut[0].dOutState = function() {};
            smartObj.dOut[0].dOutpolarity = function() {};
            smartObj.dOut[1].dOutState = function() {};
            smartObj.dOut[1].dOutpolarity = function() {};

            smartObj.dump(function (err, result) {
                if (err) 
                    done();
            });
        });
    });

    describe('#.read()', function () {
        it('should pass the read value when Resources is a primitive', function (done) {
            smartObj.read(3303, 0, 5700, function (err, result) {
                if (result === 20) 
                    done();
            });
        });

        it('should pass the read value when Resources is readable', function (done) {
            smartObj.read(3303, 0, 5701, function (err, result) {
                if (result === 'c') 
                    done();
            });
        });

        it('should pass the read value when Resources is readable and writable', function (done) {
            smartObj.read(3303, 0, 5703, function (err, result) {
                if (result === 'x') 
                    done();
            });
        });

        it('should pass the read value when restrict is true', function (done) {
            smartObj.read(3305, 0, 5800, { restrict: true }, function (err, result) {
                if (result === 20) 
                    done();
            });
        });

        it('should pass the read value when restrict is true', function (done) {
            smartObj.read(3305, 0, 5801, { restrict: true }, function (err, result) {
                if (result === 'c')
                    done();
            });
        });

        it('should pass the read value when restrict is true', function (done) {
            smartObj.read(3305, 0, 5821, { restrict: true }, function (err, result) {
                if (result === 'x') 
                    done();
            });
        });

        it('should pass _unreadable_ when restrict is true', function (done) {
            smartObj.read(3305, 0, 5806, { restrict: true }, function (err, result) {
                if (err && result === '_unreadable_') 
                    done();
            });
        });

        it('should pass _unreadable_ when Resources is writable', function (done) {
            smartObj.read(3303, 0, 5702, function (err, result) {
                if (err && result === '_unreadable_')
                    done();
            });
        });

        it('should pass _exec_ when Resources is excutable ', function (done) {
            smartObj.read(3303, 0, 5704, function (err, result) {
                if (err && result === '_exec_')
                    done();
            });
        });

        it('should pass not found error', function (done) {
            smartObj.read(3303, 0, 5706, function (err, result) {
                if (err && result === '_notfound_')
                    done();
            });
        });

        it('should pass the read value when Resources is a Multi-Resource', function (done) {
            var obj = {
                0: 'x',
                1: 'y',
                2: 'z'
            };

            smartObj.read('multiResource', 0, 5700, function (err, result) {
                if (_.isEqual(result, obj)) 
                    done();
            });
        });
    });

    describe('#.write()', function () {
        it('should write Resource and pass the write value when Resources is a primitive', function (done) {
            smartObj.write(3303, 0, 5700, 22, function (err, result) {
                if ((result === 22) && (smartObj.get(3303, 0, 5700) === 22))
                    done();
            });
        });

        it('should and pass the write value when Resources is writable', function (done) {
            smartObj.write(3303, 0, 5702, 24, function (err, result) {
                if (result === 24)
                    done();
            });
        });

        it('should pass the write value when Resources is readable and writable', function (done) {
            smartObj.write(3303, 0, 5703, 26, function (err, result) {
                if (result === 26)
                    done();
            });
        });

        it('should pass the write value when restrict is true', function (done) {
            smartObj.write(3305, 0, 5806, 'f', { restrict: true }, function (err, result) {
                if (result === 'f' && (smartObj.get(3305, 0, 5806) === 'f')) 
                    done();
            });
        });

        it('should pass the write value when restrict is true', function (done) {
            smartObj.write(3305, 0, 5821, 'y', { restrict: true }, function (err, result) {
                if (result === 'y' && (smartObj.get(3305, 0, 5821) === 'y')) 
                    done();
            });
        });

        it('should pass _unwritable_ when restrict is true', function (done) {
            smartObj.write(3305, 0, 5800, 30, { restrict: true }, function (err, result) {
                if (err && result === '_unwritable_' && (smartObj.get(3305, 0, 5800) === 20)) 
                    done();
            });
        });

        it('should pass _unwritable_ when restrict is true', function (done) {
            smartObj.write(3305, 0, 5801, 'x', { restrict: true }, function (err, result) {
                if (err && result === '_unwritable_' && (smartObj.get(3305, 0, 5801) === 'c')) 
                    done();
            });
        });

        it('should pass _unwritable_ when Resources is readable', function (done) {
            smartObj.write(3303, 0, 5701, 20, function (err, result) {
                if (result === '_unwritable_')
                    done();
            });
        });

        it('should pass _exec_ when Resources is excutable', function (done) {
            smartObj.write(3303, 0, 5704, 20, function (err, result) {
                if (result === '_exec_')
                    done();
            });
        });

        it('should pass not found error', function (done) {
            smartObj.write(3303, 0, 5706, 'C', function (err, result) {
                if (err && result === '_notfound_')
                    done();
            });
        });

        it('should write Resource and pass the write value when Resources is a Multi-Resource', function (done) {
            var obj = {
                0: 'a', 
                1: 'b', 
                2: 'c'
            };

            smartObj.write('multiResource', 0, 5700, obj, function (err, result) {
                if (_.isEqual(result, obj))
                    done();
            });
        });
    });
    
    describe('#.exec()', function () {
        it('should exec Resource and pass true through its second argument', function (done) {
            smartObj.exec(3303, 0, 5704, [ 20 ], function (err, result) {
                if (result === true)
                    done();
            });
        });

        it('should pass _unexecutable_ when Resources is a primitive', function (done) {
            smartObj.exec(3303, 0, 5700, [ 20 ], function (err, result) {
                if (result === '_unexecutable_')
                    done();
            });
        });

        it('should pass _unexecutable_ when Resources is readable', function (done) {
            smartObj.exec(3303, 0, 5701, [ 20 ], function (err, result) {
                if (result === '_unexecutable_')
                    done();
            });
        });

        it('should pass _unexecutable_ when Resources is writable', function (done) {
            smartObj.exec(3303, 0, 5702, [ 20 ], function (err, result) {
                if (result === '_unexecutable_')
                    done();
            });
        });

        it('should pass not argus error', function (done) {
            smartObj.exec(3303, 0, 5704, '20', function (err, result) {
                if (err)
                    done();
            });
        });

        it('should pass not found error', function (done) {
            smartObj.exec(3303, 0, 5706, [ 20 ], function (err, result) {
                if (err && result === '_notfound_')
                    done();
            });
        });
    });

    describe('#.isReadable()', function () {
        it('should return true if target is readable', function () {
            expect(smartObj.isReadable(3303, 0, 5700)).to.be.true;
            expect(smartObj.isReadable(3303, 0, 5701)).to.be.true;
            expect(smartObj.isReadable(3303, 0, 5703)).to.be.true;
            expect(smartObj.isReadable('3303', '0', '5700')).to.be.true;
            expect(smartObj.isReadable('temperature', 0, 'sensorValue')).to.be.true;

            expect(smartObj.isReadable(3305, 0, 5800)).to.be.true;
            expect(smartObj.isReadable(3305, 0, 5801)).to.be.true;
            expect(smartObj.isReadable(3305, 0, 5821)).to.be.true;
        });

        it('should return false if target is unreadable', function () {
            expect(smartObj.isReadable(3303, 0, 5702)).to.be.false;
            expect(smartObj.isReadable(3303, 0, 5704)).to.be.false;

            expect(smartObj.isReadable(3305, 0, 5806)).to.be.false;
            expect(smartObj.isReadable(3305, 0, 5822)).to.be.false;
        });
    });

    describe('#.isWritable()', function () {
        it('should return true if target is readable', function () {
            expect(smartObj.isWritable(3303, 0, 5702)).to.be.true;
            expect(smartObj.isWritable(3303, 0, 5703)).to.be.true;

            expect(smartObj.isWritable(3305, 0, 5806)).to.be.true;
            expect(smartObj.isWritable(3305, 0, 5821)).to.be.true;
        });

        it('should return false if target is unreadable', function () {
            expect(smartObj.isWritable(3303, 0, 5700)).to.be.false;
            expect(smartObj.isWritable(3303, 0, 5701)).to.be.false;
            expect(smartObj.isWritable(3303, 0, 5704)).to.be.false;

            expect(smartObj.isWritable(3305, 0, 5800)).to.be.false;
            expect(smartObj.isWritable(3305, 0, 5801)).to.be.false;
            expect(smartObj.isWritable(3305, 0, 5822)).to.be.false;
        });
    });

    describe('#.isExecutable()', function () {
        it('should return true if target is readable', function () {
            expect(smartObj.isExecutable(3303, 0, 5704)).to.be.true;

            expect(smartObj.isExecutable(3305, 0, 5822)).to.be.true;
        });

        it('should return false if target is unreadable', function () {
            expect(smartObj.isExecutable(3303, 0, 5700)).to.be.false;
            expect(smartObj.isExecutable(3303, 0, 5701)).to.be.false;
            expect(smartObj.isExecutable(3303, 0, 5702)).to.be.false;
            expect(smartObj.isExecutable(3303, 0, 5703)).to.be.false;

            expect(smartObj.isExecutable(3305, 0, 5800)).to.be.false;
            expect(smartObj.isExecutable(3305, 0, 5801)).to.be.false;
            expect(smartObj.isExecutable(3305, 0, 5806)).to.be.false;
            expect(smartObj.isExecutable(3305, 0, 5821)).to.be.false;
        });
    });
});
