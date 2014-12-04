var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var enabled = false;
var serialConnected = false;
var SerialPort = require("serialport").SerialPort;
var serialPort = new SerialPort("/dev/ttyACM0", {
      baudrate: 115200
});
var moment = require('moment');

serialPort.on("open", function() {
    console.log("port open");
    serialConnected = true;
    serialPort.on('data', function(data) {
        //console.log('data received: ' + data);
        if (data.toString().substring(0, 5) == "cons:") {
            writeToConsole(data.toString().substring(5).replace(/\r?\n/g, ""));
            //io.emit('console', moment().format("h:mm:ss a") + ": " + data.toString().substring(5).replace(/\r?\n/g, ""));
            //console.log("got console message: \""+ data.toString().substring(5).replace(/\r?\n/g, "") + "\"");
        }
    });
});

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

