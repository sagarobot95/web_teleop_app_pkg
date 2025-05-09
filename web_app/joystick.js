// Connect to backend WebSocket
const ws = new WebSocket('ws://localhost:8765');

const statusDiv = document.getElementById('status');
ws.onopen = function() {
    statusDiv.textContent = 'Connected to backend!';
};
ws.onerror = function(e) {
    statusDiv.textContent = 'WebSocket error.';
};
ws.onclose = function() {
    statusDiv.textContent = 'WebSocket closed.';
};

// Joystick setup
const joystick = nipplejs.create({
    zone: document.getElementById('joystick-zone'),
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: '#26a69a',
    size: 220,
    restOpacity: 0.7
});

const maxLinear = 0.22; // m/s (TurtleBot3 burger max)
const maxAngular = 2.84; // rad/s (TurtleBot3 burger max)

function sendTwist(lin, ang) {
    if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ linear: lin, angular: ang }));
    }
}

joystick.on('move', function(evt, data) {
    if (data && data.distance) {
        const angle = data.angle.radian - Math.PI / 2; // Up is forward
        const distance = Math.min(data.distance / 100, 1); // scale
        const linear = Math.cos(angle) * distance * maxLinear;
        const angular = Math.sin(angle) * distance * maxAngular;
        sendTwist(linear, angular);
    }
});

joystick.on('end', function() {
    sendTwist(0, 0);
}); 