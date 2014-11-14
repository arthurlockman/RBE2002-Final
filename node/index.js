var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var enabled = false;

app.get('/', function(req, res) {
    res.sendFile(__dirname + '/index.html');
});

io.on('connection', function(socket) {
    console.log('a user connected');
    socket.on('command', function(cmd) {
        console.log("got a command: " + cmd);
        if (cmd == 'stop') {
            stop();
        }
    });
    socket.on('enable', function(cmd) {
        if (cmd == 'enable') {
            enabled = true;
            console.log('enabling robot...');
        } else if (cmd == 'disable') {
            enabled = 'false';
            stop();
            console.log('disabling robot...');
        }
    });
});

http.listen(3000, function() {
    console.log('listening on *:3000');
});

function stop() {
    console.log('stopping robot...');
}

