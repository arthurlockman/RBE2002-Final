var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var enabled = false;
var serialConnected = false;
var SerialPort = require("serialport").SerialPort;
var serialPort = new SerialPort("/dev/ttyACM0", {
      baudrate: 115200
});
serialPort.on('error', function(data)
{
    console.log("couldn't connect to arduino");
});
var nodemailer = require('nodemailer');
var moment = require('moment');
// var random = require("node-random");

var sudo = require('sudo');
var sudo_options = {
    cachePassword: true,
    prompt: 'Password, yo: '
}

var readingCounter = 0;
var sendCounter = 1;
var imuprocess = sudo(['minimu9-ahrs', '--output', 'euler', '-b', '/dev/i2c-1'], sudo_options);
imuprocess.stderr.on('data', function(data) {
    console.log(data.toString());
});
imuprocess.stdout.on('data', function(data) {
    if (readingCounter < 400) {
        readingCounter++;
    } else if (readingCounter == 400) {
        writeToConsole("Starting IMU...");
        readingCounter++;
    } else if ((sendCounter % 1) == 0) { //slowing down data flow to make it easier for the arduino to process
        serialPort.write("imu" + parseFloat(data.toString().match(/^\s*-?\d*.\d*/g)[0]) + "\n");
        io.emit('heading', parseFloat(data.toString().match(/^\s*-?\d*.\d*/g)[0]));
        sendCounter = 1;
        //console.log("imu" + parseFloat(data.toString().match(/^\s*-?\d*.\d*/g)[0]) + "\n");
    } else {
        sendCounter++;
    }
})

var transporter = nodemailer.createTransport({
    service: 'Gmail',
    auth: {
        user: 'rbe@rthr.me',
        pass: 'hoorayrobots1'
    }
});

serialPort.on("open", function() {
    console.log("port open");
    serialConnected = true;
    serialPort.on('data', function(data) {
        data.toString().split('\n').forEach(processSerialIncoming);
        //console.log('data received: ' + data);
    });
});

function processSerialIncoming(element, index, array) {
        if (element.substring(0, 5) == "cons:") {
            writeToConsole(element.substring(5).replace(/\r?\n/g, ""));
            //io.emit('console', moment().format("h:mm:ss a") + ": " + data.toString().substring(5).replace(/\r?\n/g, ""));
            //console.log("got console message: \""+ data.toString().substring(5).replace(/\r?\n/g, "") + "\"");
        } else if (element.substring(0, 4) == "flex")
        {
            io.emit('console', 'flex');
        } else if (element.substring(0, 4) == "flfo")
        {
            io.emit('console', 'flfo');
        } else if (element.substring(0,3) == "dsp")
        {
            io.emit('dx', element.substring(3).split(',')[0]);
            io.emit('dy', element.substring(2).split(',')[1]);
            io.emit('dz', element.substring(2).split(',')[2]);
            var dx = element.substring(3).split(',')[0];
            var dy = element.substring(2).split(',')[1];
            var dz = element.substring(2).split(',')[2];
            var mailOptions = {
                from: 'The Last Airbender <rbe@rthr.me>', // sender address
                to: 'ajlockman@wpi.edu, tchaydon@wpi.edu, ltutt@wpi.edu', // list of receivers
                subject: 'Fire', // Subject line
                text: 'Dear Sir/Madam,\n\nFire! Fire! Help me! Location ('+dx+'in, '+dy+'in, '+dz+'in)\n\nLooking forward to hearing from you!\n\nAll the best,\n\n-The Last Airbender', // plaintext body
                html: '<p>Dear Sir/Madam,</p><p>Fire! Fire! Help me! <b>Location ('+dx+'in, '+dy+'in, '+dz+'in)</b>.</p><p>Looking forward to hearing from you!</p><p>All the best,</p><p>-The Last Airbender</p>' // html body
            };

            transporter.sendMail(mailOptions, function(error, info){
                if(error) {
                    console.log(error);
                } else {
                    console.log('Message sent: ' + info.response);
                }
            });
        } else {
            console.log(element);
        }
}

function writeToConsole(message) {
    io.emit('console', moment().format("h:mm:ss a") + ": " + message);
    console.log(moment().format("h:mm:ss a") + ": " +message);
}

app.get('/', function(req, res) {
    res.sendFile(__dirname + '/index.html');
});

io.on('connection', function(socket) {
    console.log('a user connected');
    io.emit('console', 'hello user!');
    socket.on('command', function(cmd) {
        console.log("got a command: " + cmd);
        if (cmd == 'stop') {
            stop();
        } else {
            drive(cmd);
        }
    });
    socket.on('enable', function(cmd) {
        if (cmd == 'enable') {
            enabled = true;
            serialPort.write('en\n');
            // random.numbers({
            //     "number": 1,
            //     "minimum": 1,
            //     "maximum": 10000
            // }, function(error, data) {
            //     if (error) throw error;
            //     console.log("Random seed: " + data);
            //     serialPort.write("ran" + data + "\n");
            // });
            console.log('enabling robot...');
        } else if (cmd == 'disable') {
            enabled = false;
            stop();
            serialPort.write('ds\n');
            serialPort.write('fo\n');
            console.log('disabling robot...');
        }
    });
    socket.on('disconnect', function() {
        console.log('Controller disconnected. Disabling...');
        stop();
        enabled = false;
        serialPort.write('fo\n');
        serialPort.write('ds\n');
    });
});

http.listen(3000, function() {
    console.log('listening on *:3000');
});

function stop() {
    console.log('stopping robot...');
    serialPort.write('st\n');
}

function drive(direction) {
    if (enabled) {
        serialPort.write(direction + '\n');
        console.log('writing ' + direction);
    }
}

