
var CoapNode = require('coap-node');  //LWM2M nodes
var SmartObject = require('smartobject');
var shortid = require('shortid'); //URI Generation
var fs = require('fs'); //File system

var so = new SmartObject;
var ID = shortid.generate();
var cnode = new CoapNode('Rot_' + ID , so);


// Config parameters
var ip = process.argv[2],
    bs = false,
    status = false,
    latitude = 0, // have some random longitude and latitude
    longitude = 0,
    mode,
    RefreshStep;

// Command line arguments
process.argv.forEach(function (val) { // for every element you input
    if (val === '-t') {
        mode = val;
    }
    // Bootstrap
    if (val === '-b') {
        var http = require('http');
        bs = true;

        // Security Object
        so.init(0, 0, {0: 'coap://'+ip+':5683', 1: true, 2: 3});
        so.init(0, 1, {0: '', 1: false, 2: 3, 3: '', 4: '', 5: '', 6: 3, 7: '', 8: '', 9: '', 10: 0, 11: 0});

        // Server Object
        so.init(1, 1, {0: 1, 1: cnode.lifetime, 2: cnode._config.defaultMinPeriod, 3: cnode._config.defaultMaxPeriod, 6: false, 7: 'U'});

        // Send bootstrap information to BS Server
        var options = {
          hostname: ip,
          port: 8080,
          path: '/api/bootstrap/' + cnode.clientName,
          method: 'POST',
          headers: {
            'content-type': 'application/json'
          }
        };

        var stream = fs.createReadStream('./data.json');
        var req = http.request(options, function(res) {
          // Send bootstrap request
          cnode.bootstrap(ip, 5683, function (err, rsp) {
              if (err) {
                console.log(err);
              }
          });
        });

        req.on('error', (e) => {
          console.log(`problem with request: ${e.message}`);
        });
        stream.pipe(req);
    }
});

// Server Object
so.init(1, 0, {
  0: 0,                                                                         // ServerID
  1: cnode.lifetime,                                                            // Lifetime
  2: cnode._config.defaultMinPeriod,
  3: cnode._config.defaultMaxPeriod,
  6: false,
  7: 'U',
  8: {                                                                          // Update
    exec: function(attrs, cb) {
      update(attrs);
      cb(null);
    }
  }
});

// Device Object
so.init(3, 0, {
  0: 'Manu',              		                                                // Manufactorer name
  1: getRandomInt(1, 100).toString(),                                           // Model number
  5: {                                                                          // Reset
    exec: function(cb) {
      reset();
      cb(null);
    }
  },
  16: 'U'                                                                       // Binding mode
});

// Location Object
so.init(6, 0, {
  0: latitude,                                                                  // Latitude
  1: longitude,                                                                 // Longitude
  5: Math.floor(Date.now() / 1000)                                              // Timestamp
});

// Connection Monitoring
so.init(4, 0, {
  4: cnode.ip,
  5: 'unknown'
});


// Update
function update(attrs) {
  // Set lifetime, version attributes
  cnode.update({lifetime: attrs}, {version: '1.0.0'}, function (err, rsp) {
    if(err) {
      console.log(err);
    }
  });
}

// Factory reset
function reset(callback) {
  if (bs === false) {
    so.set('1', 0, '1', 86000);
    so.set('1', 0, '2', 10);
    so.set('1', 0, '3', 60);

    // Delete all extra object instances
    var obj = cnode.getSmartObject();

    // Delete all extra object instances
    var obj = cnode.getSmartObject();
    for (var i=0, item; item = obj.objectList()[i]; i++) {
        for (var j=1; item.iid[j]; j++) {
            cnode.deleteInst(item.oid, item.iid[j]);
        }
    }
    //De-register
    cnode.register(ip, 5683, function (err, rsp) {
        if (err) {
          console.log(err);
        }
    });

  } else {
    // Re-bootstrap
    ip = so.get('0', 0, '0');
    ip = ip.substring(7, ip.length);
    ip = ip.substring(0, ip.indexOf(':'));
    cnode.bootstrap(ip, 5683, function (err, rsp) {
        if (err) {
          console.log(err);
        }
    });
  }
}

// Support functions

function find(mode) {
  // console.log("find Activated");
  if (mode === '-t')  {
    latitude = (Math.round(getRandomArbitrary(-90, 90) * 10) / 100).toString(); // have some random longitude and latitude
    longitude = (Math.round(getRandomArbitrary(-180, 180) * 10) / 100).toString();

    so.write (6, 0, 0,latitude, function (err, data) {
          if (err)
              console.log(err);  // 1
      });
    so.read (6, 0, 0, function (err, data){
        if (!err)
            console.log("Random latitude: " + data);
      });
      so.write (6, 0, 1, longitude, function (err, data) {
          if (err)
              console.log(err);  // 1
      });
      so.read (6, 0, 1, function (err, data){
          if (!err)
              console.log("Random longitude: " + data);
      });}

        else {console.log('Sensor with random data');}
};

function RefreshFind(mode){
  // console.log("RefreshFind Activated");
  RefreshStep = setInterval(find, 10000, "-t");
  }


// Random float number generator
function getRandomArbitrary(min, max) {
  return Math.random() * (max - min) + min;
};

// Random integer
function getRandomInt(min, max) {
  return Math.floor(Math.random() * (max - min)) + min;
};


// Trap signals -> de-register
process.on('SIGTERM', function() {
  process.stdout.write('\n');
  cnode.deregister(function (err, rsp) {
      if (err) {
        console.log(err);
      }
      process.exit(0);
  });
});

process.on('SIGINT', function() {
  process.stdout.write('\n');
  cnode.deregister(function (err, rsp) {
      if (err) {
        console.log(err);
      }
      process.exit(0);
  });
});

// This event fired when the device registered (2.01).
cnode.on('registered', function () {
  console.log('registered' + ' Rot_' + ID);
});


// This event fired when there is an error occurred.
cnode.on('error', function(err, rsp) {
    console.log(err);
});

// Multicast
cnode.on('multicast', function() {
    console.log('multicast');
});

// This event fired when the device attributes updated (2.04).
cnode.on('updated', function (err, rsp) {
    console.log('updated');
});

// This event fired when the device de-registered (2.02).
cnode.on('deregistered', function () {
    console.log('deregistered' + ' Rot_' + ID);
});

// This event fired when the bootstrapped (2.02).
cnode.on('bootstrapped', function () {
    console.log('bootstrapped');

    ip = so.get('0', 1, '0');
    ip = ip.substring(7, ip.length);
    ip = ip.substring(0, ip.indexOf(':'));

    // Find sensoe data;
    find(mode);

    // Register
    cnode.register(ip, 5683, function (err, rsp) {
        if (err) {
          console.log(err);
        }
        else{
          console.log("Register_rsp = " + rsp);
          }
    });
});

// No BS server
if (bs === false) {

    so.init(0, 0, { 0: 'coap://' + ip + ':5683', 1: false, 2: 3});

    RefreshFind(mode);

    cnode.register(ip, 5683, function (err, rsp) {
        if (err) {
          console.log(err);
        }
        else{
          console.log("Hi"+rsp);
        }
    });
}
