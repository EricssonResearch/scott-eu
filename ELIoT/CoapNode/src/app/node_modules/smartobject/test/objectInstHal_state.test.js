var _ = require('busyman'),
    expect = require('chai').expect, 
    SmartObject = require('../index.js'); 

var hal = {
    led1: 'led1',
    led2: 'led2'
};

var innerHalRef;
var innerHal =  {
    led3: 'led3',
    led4: 'led4'
};

var smartObj1 = new SmartObject(hal);
var smartObj2 = new SmartObject(innerHal, function () {
    innerHalRef = this.hal;
    this.hal.x = 0;
});

smartObj1.init('test', 0, {
    r1: 1,
    r2: 2
});

var resrc2 = {
    _state: {
        state1: 'state1',
        state2: 'state2'
    },
    r1: 1,
    r2: 2
};

smartObj2.init('test', 0, resrc2);

describe('Smart Object - Hal', function () {
    describe('#smartObj1 this.hal', function () {
        it('should equal to hal', function () {
            expect(smartObj1.hal).to.be.equal(hal);
            expect(smartObj1.hal.led1).to.be.equal('led1');
            expect(smartObj1.hal.led2).to.be.equal('led2');
        });
    });

    describe('#smartObj2 this.hal', function () {
        it('should equal to innerHal', function () {
            expect(smartObj2.hal).to.be.equal(innerHalRef);
            expect(smartObj2.hal.led3).to.be.equal('led3');
            expect(smartObj2.hal.led4).to.be.equal('led4');
            expect(smartObj2.hal.x).to.be.equal(0);
        });
    });
});

describe('Smart Object Instance - Resource', function () {
    describe('#smartObj1.test instance: this.parent.hal', function () {
        it('should equal to hal', function () {
            var inst = smartObj1.findObjectInstance('test', 0);
            expect(inst.parent.hal).to.be.equal(hal);
        });

        it('r1 should equal to 1', function () {
            var inst = smartObj1.findObjectInstance('test', 0);
            expect(inst.r1).to.be.equal(1);
        });

        it('r2 should equal to 2', function () {
            var inst = smartObj1.findObjectInstance('test', 0);
            expect(inst.r2).to.be.equal(2);
        });
    });
});

describe('Smart Object Instance - _state', function () {
    describe('#smartObj2.test instance: this._state', function () {
        it('state1 should equal to state1', function () {
            var inst = smartObj2.findObjectInstance('test', 0);
            expect(inst._state.state1).to.be.equal('state1');
        });

        it('state1 should equal to state2', function () {
            var inst = smartObj2.findObjectInstance('test', 0);
            expect(inst._state.state2).to.be.equal('state2');
        });
    });
});

describe('Smart Object Instance - init then clear', function () {
    describe('#smartObj2.test instance: init', function () {
        it('state1 should equal to state1', function () {
            var inst = smartObj2.findObjectInstance('test', 0);

            smartObj2.init('test', 0, {});
            expect(inst._state.state1).to.be.undefined;
        });

        it('state1 should equal to state2', function () {
            var inst = smartObj2.findObjectInstance('test', 0);

            smartObj2.init('test', 0, {});
            expect(inst._state.state2).to.be.undefined;
        });
    });
});
