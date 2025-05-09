import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';
import nipplejs from 'nipplejs';

let ws = null;
let joystick = null;
let isKeyboardControl = false;
let keyboardState = {
    forward: false,
    backward: false,
    left: false,
    right: false
};

// Initialize WebSocket connection
function initWebSocket() {
    ws = new WebSocket('ws://localhost:8765');
    
    ws.onopen = () => {
        document.getElementById('status').textContent = 'Connected to ROS';
        // Request robot description
        ws.send(JSON.stringify({ type: 'get_robot_description' }));
        // Start odometry updates
        requestOdomUpdate();
    };
    
    ws.onclose = () => {
        document.getElementById('status').textContent = 'Disconnected from ROS';
        setTimeout(initWebSocket, 1000);
    };
    
    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        if (data.type === 'robot_description') {
            initURDFViewer(data.data);
        } else if (data.type === 'odom') {
            updateOdomDisplay(data.data);
        }
    };
}

// Initialize joystick
function initJoystick() {
    const options = {
        zone: document.getElementById('joystick-zone'),
        mode: 'static',
        position: { left: '50%', top: '50%' },
        color: '#00796b'
    };
    
    joystick = nipplejs.create(options);
    
    joystick.on('move', (evt, data) => {
        if (!isKeyboardControl) {
            const linear = data.vector.y * -0.5;  // Scale down for better control
            const angular = data.vector.x * 0.5;
            sendCommand(linear, angular);
        }
    });
    
    joystick.on('end', () => {
        if (!isKeyboardControl) {
            sendCommand(0, 0);
        }
    });
}

// Initialize URDF viewer
function initURDFViewer(urdfData) {
    const container = document.getElementById('urdf-viewer');
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);
    
    // Add lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
    directionalLight.position.set(1, 1, 1);
    scene.add(directionalLight);
    
    // Load URDF
    const loader = new URDFLoader();
    const robot = loader.parse(urdfData);
    scene.add(robot);
    
    // Position camera
    camera.position.set(0.5, 0.5, 0.5);
    camera.lookAt(0, 0, 0);
    
    // Animation loop
    function animate() {
        requestAnimationFrame(animate);
        renderer.render(scene, camera);
    }
    animate();
}

// Update odometry display
function updateOdomDisplay(odomData) {
    document.getElementById('pos-x').textContent = odomData.position.x.toFixed(2);
    document.getElementById('pos-y').textContent = odomData.position.y.toFixed(2);
    document.getElementById('pos-z').textContent = odomData.position.z.toFixed(2);
    document.getElementById('lin-vel').textContent = odomData.linear_velocity.x.toFixed(2);
    document.getElementById('ang-vel').textContent = odomData.angular_velocity.z.toFixed(2);
}

// Request odometry updates
function requestOdomUpdate() {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'get_odom' }));
        setTimeout(requestOdomUpdate, 100);  // Update every 100ms
    }
}

// Send command to robot
function sendCommand(linear, angular) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({
            linear: linear,
            angular: angular
        }));
    }
}

// Handle keyboard controls
function initKeyboardControls() {
    document.addEventListener('keydown', (e) => {
        if (!isKeyboardControl) return;
        
        switch(e.key.toLowerCase()) {
            case 'w':
                keyboardState.forward = true;
                break;
            case 's':
                keyboardState.backward = true;
                break;
            case 'a':
                keyboardState.left = true;
                break;
            case 'd':
                keyboardState.right = true;
                break;
            case ' ':
                sendCommand(0, 0);
                break;
        }
        updateKeyboardMovement();
    });
    
    document.addEventListener('keyup', (e) => {
        if (!isKeyboardControl) return;
        
        switch(e.key.toLowerCase()) {
            case 'w':
                keyboardState.forward = false;
                break;
            case 's':
                keyboardState.backward = false;
                break;
            case 'a':
                keyboardState.left = false;
                break;
            case 'd':
                keyboardState.right = false;
                break;
        }
        updateKeyboardMovement();
    });
}

// Update movement based on keyboard state
function updateKeyboardMovement() {
    if (!isKeyboardControl) return;
    
    let linear = 0;
    let angular = 0;
    
    if (keyboardState.forward) linear += 0.5;
    if (keyboardState.backward) linear -= 0.5;
    if (keyboardState.left) angular += 0.5;
    if (keyboardState.right) angular -= 0.5;
    
    sendCommand(linear, angular);
}

// Initialize control toggle
function initControlToggle() {
    const toggleButton = document.getElementById('control-toggle');
    const keyboardControls = document.getElementById('keyboard-controls');
    
    toggleButton.addEventListener('click', () => {
        isKeyboardControl = !isKeyboardControl;
        toggleButton.textContent = isKeyboardControl ? 'Switch to Joystick Control' : 'Switch to Keyboard Control';
        keyboardControls.classList.toggle('active');
        
        // Reset movement when switching controls
        sendCommand(0, 0);
        if (joystick) {
            joystick.destroy();
            if (!isKeyboardControl) {
                initJoystick();
            }
        }
    });
}

// Initialize everything
function init() {
    initWebSocket();
    initJoystick();
    initKeyboardControls();
    initControlToggle();
}

// Start the application
init(); 