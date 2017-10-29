var _ = require('busyman'),
    expect = require('chai').expect, 
    SmartObject = require('../index.js'); 

var smartObj = new SmartObject();

describe('Smart Object - Instance read/write/exec cb bind this to instance itself', function () {
    describe('#read', function () {
        it('should bind this to instance itself in read method', function (done) {
            smartObj.init(3303, 0, { 
                a: 'hello world',
                5700: {
                    read: function (cb) {
                        cb(null, this);
                    }    
                }
            });

            smartObj.read(3303, 0, 5700, function (err, val) {
                if (val === smartObj['temperature'][0])
                    done();
            });
        });
    });

    describe('#write', function () {
        it('should bind this to instance itself in write method', function (done) {
            smartObj.init('positioner', 1, { 
                5536: {
                    write: function (val, cb) {
                        cb(null, this);
                    }
                }
            });

            smartObj.write('positioner', 1, 5536, 10, function (err, val) {
                if (val === smartObj['positioner'][1])
                    done();
            });
        });
    });

    describe('#exec', function () {
        it('should bind this to instance itself in exec method', function (done) {
            smartObj.init('positioner', 2, { 
                5605: {
                    exec: function (cb) {
                        cb(null, this);
                    }
                }
            });

            smartObj.exec('positioner', 2, 5605, [], function (err, val) {
                if (val === smartObj['positioner'][2])
                    done();
            });
        });
    });
});