// AMR Web GUI Application Logic

// DOM Elements
const connectionStatus = document.getElementById('connection-status');
const statusText = document.getElementById('status-text');
const batteryPercentage = document.getElementById('battery-percentage');
const batteryBarFill = document.getElementById('battery-bar-fill');
const batteryIconState = document.getElementById('battery-icon-state');
const navButtons = document.querySelectorAll('.nav-btn');
const viewSections = document.querySelectorAll('.view-section');
const rackListContainer = document.getElementById('rack-list-container');

// ROS Setup
const ros = new ROSLIB.Ros();

// Connect to ROSBridge
function connectToROS() {
    ros.connect('ws://' + window.location.hostname + ':9090');
}

ros.on('connection', () => {
    console.log('Connected to websocket server.');
    connectionStatus.className = 'status-indicator connected';
    statusText.innerText = 'ONLINE';
    initROSComponents();
});

ros.on('error', (error) => {
    console.log('Error connecting to websocket server: ', error);
    connectionStatus.className = 'status-indicator disconnected';
    statusText.innerText = 'ERROR';
});

ros.on('close', () => {
    console.log('Connection to websocket server closed.');
    connectionStatus.className = 'status-indicator disconnected';
    statusText.innerText = 'OFFLINE';
    // Try to reconnect every 3 seconds
    setTimeout(connectToROS, 3000);
});

// View Navigation
navButtons.forEach(btn => {
    btn.addEventListener('click', (e) => {
        // Remove active class from all buttons and sections
        navButtons.forEach(b => b.classList.remove('active'));
        viewSections.forEach(s => s.classList.remove('active', 'hidden'));
        viewSections.forEach(s => s.classList.add('hidden'));
        
        // Add active class to clicked button and target section
        btn.classList.add('active');
        const targetId = btn.getAttribute('data-target');
        document.getElementById(targetId).classList.remove('hidden');
        document.getElementById(targetId).classList.add('active');
    });
});

// Initialize ROS Publishers/Subscribers
function initROSComponents() {
    // 1. Battery Subscriber
    const batterySub = new ROSLIB.Topic({
        ros: ros,
        name: '/battery_level',
        messageType: 'sensor_msgs/msg/BatteryState'
    });

    batterySub.subscribe((msg) => {
        // Fix: Multiply by 100 to get a 0-100 scale
        const percentage = Math.round(msg.percentage * 100);
        batteryPercentage.innerText = `${percentage}%`;
        batteryBarFill.style.width = `${percentage}%`;
        
        if (percentage <= 20) {
            batteryBarFill.style.background = 'var(--accent-neon-red)';
            batteryBarFill.style.boxShadow = '0 0 10px var(--accent-neon-red)';
            batteryIconState.className = 'fa-solid fa-battery-empty';
            batteryIconState.style.color = 'var(--accent-neon-red)';
        } else if (percentage <= 50) {
            batteryBarFill.style.background = 'var(--accent-neon-orange)';
            batteryBarFill.style.boxShadow = '0 0 10px var(--accent-neon-orange)';
            batteryIconState.className = 'fa-solid fa-battery-half';
            batteryIconState.style.color = 'var(--accent-neon-orange)';
        } else {
            batteryBarFill.style.background = 'var(--accent-neon-green)';
            batteryBarFill.style.boxShadow = '0 0 10px var(--accent-neon-green)';
            batteryIconState.className = 'fa-solid fa-battery-full';
            batteryIconState.style.color = 'var(--accent-neon-green)';
        }
    });

    // 2. Teleoperation Publisher (cmd_vel)
    const cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist'
    });

    // Initialize Virtual Joystick
    const joystickZone = document.getElementById('joystick-zone');
    const manager = nipplejs.create({
        zone: joystickZone,
        mode: 'static',
        position: { left: '50%', top: '50%' },
        color: '#ff5500',
        size: 150
    });

    manager.on('move', (event, data) => {
        const max_linear = 0.5; // m/s
        const max_angular = 1.0; // rad/s
        const max_distance = 75.0; // max size of joystick

        const linear_speed = Math.sin(data.angle.radian) * max_linear * data.distance / max_distance;
        const angular_speed = -Math.cos(data.angle.radian) * max_angular * data.distance / max_distance;

        const twist = new ROSLIB.Message({
            linear: { x: linear_speed, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: angular_speed }
        });
        cmdVel.publish(twist);
    });

    manager.on('end', () => {
        // Stop robot when joystick released
        const twist = new ROSLIB.Message({
            linear: { x: 0.0, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: 0.0 }
        });
        cmdVel.publish(twist);
    });

    // E-STOP Publisher for Backend
    const estopPub = new ROSLIB.Topic({
        ros: ros,
        name: '/gui/estop',
        messageType: 'std_msgs/msg/Empty'
    });

    // E-STOP Button
    document.getElementById('btn-estop').addEventListener('click', () => {
        const twist = new ROSLIB.Message({
            linear: { x: 0.0, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: 0.0 }
        });
        cmdVel.publish(twist);
        estopPub.publish(new ROSLIB.Message({}));
        console.log("E-STOP triggered (0-velocity and process kill signal sent).");
    });

    // 3. 2D Map Visualization (Custom Canvas Renderer with Live Robot Location)
    const mapCanvas = document.getElementById('map-canvas');
    const ctx = mapCanvas.getContext('2d');
    let offscreenCanvas = document.createElement('canvas');
    let offscreenCtx = offscreenCanvas.getContext('2d');
    
    // Store map info globally so the odom subscriber can use it
    let currentMapInfo = null;
    let currentMapScale = 1;

    const mapSub = new ROSLIB.Topic({
        ros: ros,
        name: '/map',
        messageType: 'nav_msgs/msg/OccupancyGrid'
    });

    mapSub.subscribe((msg) => {
        currentMapInfo = msg.info;
        const width = msg.info.width;
        const height = msg.info.height;
        
        // Update canvas size if it changed
        if (offscreenCanvas.width !== width || offscreenCanvas.height !== height) {
            offscreenCanvas.width = width;
            offscreenCanvas.height = height;
        }

        const imgData = offscreenCtx.createImageData(width, height);
        const data = msg.data;

        for (let i = 0; i < data.length; i++) {
            const val = data[i];
            const pxIndex = i * 4;

            if (val === -1) {
                // Unknown (Transparent)
                imgData.data[pxIndex] = 0;
                imgData.data[pxIndex + 1] = 0;
                imgData.data[pxIndex + 2] = 0;
                imgData.data[pxIndex + 3] = 0;
            } else if (val === 100) {
                // Obstacle / Wall (Neon Green)
                imgData.data[pxIndex] = 57;
                imgData.data[pxIndex + 1] = 255;
                imgData.data[pxIndex + 2] = 20;
                imgData.data[pxIndex + 3] = 255;
            } else if (val === 0) {
                // Free space (Dark Gray)
                imgData.data[pxIndex] = 25;
                imgData.data[pxIndex + 1] = 25;
                imgData.data[pxIndex + 2] = 25;
                imgData.data[pxIndex + 3] = 255;
            } else {
                // Unknown/Intermediate (Grey)
                imgData.data[pxIndex] = 80;
                imgData.data[pxIndex + 1] = 80;
                imgData.data[pxIndex + 2] = 80;
                imgData.data[pxIndex + 3] = 255;
            }
        }
        
        // Paint offscreen canvas
        offscreenCtx.putImageData(imgData, 0, 0);

        // Scale and draw to main canvas
        currentMapScale = Math.min(
            document.getElementById('map-container').clientWidth / width,
            document.getElementById('map-container').clientHeight / height
        ) * 0.9; // 90% to leave a tiny padding

        mapCanvas.width = width * currentMapScale;
        mapCanvas.height = height * currentMapScale;

        redrawMapAndRobot(null); // Draw just the map initially
    });

    // Helper to draw map and then the robot on top
    function redrawMapAndRobot(robotPose) {
        if (!currentMapInfo) return;

        // Flip vertically (ROS map origin is bottom left, canvas is top left)
        ctx.save();
        ctx.scale(1, -1);
        ctx.translate(0, -mapCanvas.height);
        
        ctx.imageSmoothingEnabled = false; // Keep the pixelated sci-fi look
        ctx.drawImage(offscreenCanvas, 0, 0, mapCanvas.width, mapCanvas.height);
        
        // Draw Robot if we have a pose
        if (robotPose) {
            const res = currentMapInfo.resolution;
            const originX = currentMapInfo.origin.position.x;
            const originY = currentMapInfo.origin.position.y;

            // Convert world coords to map pixels
            const px = ((robotPose.x - originX) / res) * currentMapScale;
            const py = ((robotPose.y - originY) / res) * currentMapScale;

            // Draw a Sci-Fi Cyan Triangle for the robot
            ctx.translate(px, py);
            ctx.rotate(robotPose.yaw);
            
            ctx.beginPath();
            ctx.moveTo(10, 0); // Nose
            ctx.lineTo(-6, -6); // Bottom right
            ctx.lineTo(-4, 0); // Back indent
            ctx.lineTo(-6, 6); // Bottom left
            ctx.closePath();
            
            ctx.fillStyle = '#00ffff'; // Neon Cyan
            ctx.fill();
            ctx.shadowBlur = 10;
            ctx.shadowColor = '#00ffff';
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 1;
            ctx.stroke();
        }
        
        ctx.restore();
    }

    // Subscribe to Odometry for live robot position
    const odomSub = new ROSLIB.Topic({
        ros: ros,
        name: '/odom',
        messageType: 'nav_msgs/msg/Odometry'
    });

    odomSub.subscribe((msg) => {
        // Convert quaternion to Euler yaw
        const q = msg.pose.pose.orientation;
        const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        const yaw = Math.atan2(siny_cosp, cosy_cosp);

        const robotPose = {
            x: msg.pose.pose.position.x,
            y: msg.pose.pose.position.y,
            yaw: yaw
        };
        
        redrawMapAndRobot(robotPose);
    });

    // 4. Mission Triggers
    const triggerMappingPub = new ROSLIB.Topic({
        ros: ros,
        name: '/gui/trigger_mapping',
        messageType: 'std_msgs/msg/Empty'
    });

    document.getElementById('btn-start-mapping').addEventListener('click', () => {
        triggerMappingPub.publish(new ROSLIB.Message({}));
        console.log("Triggered mapping sequence.");
    });

    const triggerOrchestrationPub = new ROSLIB.Topic({
        ros: ros,
        name: '/gui/trigger_orchestration',
        messageType: 'std_msgs/msg/Empty'
    });

    document.getElementById('btn-start-orchestration').addEventListener('click', () => {
        triggerOrchestrationPub.publish(new ROSLIB.Message({}));
        console.log("Triggered orchestration sequence.");
    });

    // 5. Rack Status Setup
    setupRackDemo();
}

function setupRackDemo() {
    let globalRacks = [];

    // Initial display
    renderRacks([{
        id: 'AWAITING_DATA',
        location: 'Scanning map for targets...',
        checked: false
    }]);

    // Subscribe to the actual rack poses detected by map_operations.py
    const rackPosesSub = new ROSLIB.Topic({
        ros: ros,
        name: '/rack_poses',
        messageType: 'geometry_msgs/msg/PoseArray'
    });

    rackPosesSub.subscribe((msg) => {
        globalRacks = [];
        msg.poses.forEach((pose, index) => {
            globalRacks.push({
                id: `RACK-${(index + 1).toString().padStart(3, '0')}`,
                location: `X: ${pose.position.x.toFixed(2)}, Y: ${pose.position.y.toFixed(2)}`,
                checked: false
            });
        });
        renderRacks(globalRacks);
        
        // Only render once when we get the array
        rackPosesSub.unsubscribe();
    });

    // Subscribe to rosout to catch Behavior Tree logs!
    const rosoutSub = new ROSLIB.Topic({
        ros: ros,
        name: '/rosout',
        messageType: 'rcl_interfaces/msg/Log'
    });

    rosoutSub.subscribe((msg) => {
        if (msg.msg && msg.msg.includes("Popped a rack. Remaining:")) {
            // Extract the remaining number
            const match = msg.msg.match(/Remaining:\s*(\d+)/);
            if (match && globalRacks.length > 0) {
                const remaining = parseInt(match[1]);
                const completedCount = globalRacks.length - remaining;
                
                // Mark the first 'completedCount' racks as checked
                let changed = false;
                for(let i = 0; i < globalRacks.length; i++) {
                    const shouldBeChecked = i < completedCount;
                    if(globalRacks[i].checked !== shouldBeChecked) {
                        globalRacks[i].checked = shouldBeChecked;
                        changed = true;
                    }
                }
                
                if(changed) {
                    renderRacks(globalRacks);
                }
            }
        }
    });
}

function renderRacks(racks) {
    rackListContainer.innerHTML = '';
    racks.forEach(rack => {
        const li = document.createElement('li');
        li.className = `rack-item ${rack.checked ? 'checked' : ''}`;
        
        li.innerHTML = `
            <div>
                <div class="rack-id">${rack.id}</div>
                <div style="font-size: 12px; color: var(--text-secondary); margin-top: 5px;">${rack.location}</div>
            </div>
            <i class="fa-solid ${rack.checked ? 'fa-check-double' : 'fa-spinner fa-spin'} rack-status-icon"></i>
        `;
        rackListContainer.appendChild(li);
    });
}

// Start connection attempt on load
window.onload = () => {
    connectToROS();
};
