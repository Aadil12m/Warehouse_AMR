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
const rackSummary = document.getElementById('rack-summary');
const missionPhase = document.getElementById('mission-phase');
const missionLog = document.getElementById('mission-log');
const actionButtons = document.querySelectorAll('.action-btn');
const mapCanvas = document.getElementById('map-canvas');
const mapContainer = document.getElementById('map-container');

// ---------------------------------------------------------------- ROS connection
const ros = new ROSLIB.Ros();

// Everything that belongs to ONE websocket connection. Recreated on every
// (re)connect: roslibjs does not re-send subscriptions over a new socket, and
// the UI (joystick, buttons) must NOT be set up again - that used to create a
// second joystick and make every button click publish once per reconnect.
let conn = null;

function connectToROS() {
    ros.connect('ws://' + window.location.hostname + ':9090');
}

ros.on('connection', () => {
    console.log('Connected to websocket server.');
    connectionStatus.className = 'status-indicator connected';
    statusText.innerText = 'ONLINE';
    setButtonsEnabled(true);
    openConnection();
    addLog('gui', 'Connected to rosbridge.');
});

ros.on('error', (error) => {
    console.log('Error connecting to websocket server: ', error);
    connectionStatus.className = 'status-indicator disconnected';
    statusText.innerText = 'ERROR';
});

let reconnectTimer = null;
ros.on('close', () => {
    console.log('Connection to websocket server closed.');
    connectionStatus.className = 'status-indicator disconnected';
    statusText.innerText = 'OFFLINE';
    setButtonsEnabled(false);
    closeConnection();
    // Try to reconnect every 3 seconds
    if (!reconnectTimer) {
        reconnectTimer = setTimeout(() => { reconnectTimer = null; connectToROS(); }, 3000);
    }
});

function setButtonsEnabled(enabled) {
    actionButtons.forEach(b => { b.disabled = !enabled; });
}

function openConnection() {
    closeConnection();
    conn = { topics: {}, listenedTopics: [] };

    subscribeBattery();
    subscribeMap();
    subscribeOdom();
    subscribeExploreStatus();
    // Latched one-shot topic: explicit transient_local QoS (see subscribeRaw).
    subscribeRaw('/rack_poses', 'geometry_msgs/msg/PoseArray',
        { reliability: 'reliable', durability: 'transient_local', history: 'keep_last', depth: 1 },
        onRackPoses);
    // Live log only: rosout publishers are transient_local, so the default would
    // replay every node's old messages (e.g. a previous run's "Mission complete").
    subscribeRaw('/rosout', 'rcl_interfaces/msg/Log',
        { reliability: 'reliable', durability: 'volatile', history: 'keep_last', depth: 100 },
        onRosout);
}

function closeConnection() {
    if (!conn) return;
    // Drop message listeners only; the old socket's subscriptions died with it.
    conn.listenedTopics.forEach(name => ros.removeAllListeners(name));
    conn = null;
}

function subscribe(name, messageType, callback, options = {}) {
    const topic = new ROSLIB.Topic(Object.assign({ ros, name, messageType }, options));
    topic.subscribe(callback);
    conn.listenedTopics.push(name);
    return topic;
}

// ROSLIB.Topic cannot request a QoS, and rosbridge then only picks
// transient_local if a matching publisher already exists when we subscribe.
// Send the subscribe op ourselves (rosbridge on Jazzy honours "qos");
// rosbridge still emits incoming messages on `ros` under the topic name.
function subscribeRaw(name, type, qos, callback) {
    ros.callOnConnection({ op: 'subscribe', id: `subscribe:${name}:qos`, topic: name, type, qos });
    ros.on(name, callback);
    conn.listenedTopics.push(name);
}

function publish(name, messageType, msg, quiet = false) {
    if (!conn) {
        if (!quiet) addLog('gui', `Not connected: ${name} not sent.`, 'warn');
        return false;
    }
    if (!conn.topics[name]) {
        conn.topics[name] = new ROSLIB.Topic({ ros, name, messageType });
    }
    conn.topics[name].publish(new ROSLIB.Message(msg));
    return true;
}

// ---------------------------------------------------------------- Mission phase + log
const PHASE_TONES = ['idle', 'active', 'ok', 'warn', 'danger'];
function setPhase(text, tone) {
    missionPhase.innerText = text;
    PHASE_TONES.forEach(t => missionPhase.classList.remove(t));
    missionPhase.classList.add(tone);
}

// Nodes whose /rosout messages are mission-relevant (others, e.g. costmaps, are noise).
const LOG_NODE_PATTERN = /^(bt_mission_controller|explore_node|map_operation_node|map_saver|gui_bridge_node|bt_navigator)/;
const MAX_LOG_LINES = 80;

function addLog(source, text, level = 'info') {
    const li = document.createElement('li');
    li.className = `log-line ${level}`;
    const time = new Date().toLocaleTimeString([], { hour12: false });
    li.innerHTML = `<span class="log-time">${time}</span><span class="log-src"></span><span class="log-msg"></span>`;
    li.querySelector('.log-src').textContent = source;
    li.querySelector('.log-msg').textContent = text;
    const atBottom = missionLog.scrollHeight - missionLog.scrollTop - missionLog.clientHeight < 30;
    missionLog.appendChild(li);
    while (missionLog.children.length > MAX_LOG_LINES) missionLog.removeChild(missionLog.firstChild);
    if (atBottom) missionLog.scrollTop = missionLog.scrollHeight;
}

// rcl_interfaces/msg/Log levels
const LOG_LEVELS = { 10: 'debug', 20: 'info', 30: 'warn', 40: 'error', 50: 'error' };

let mapSavedThisSession = false;

function onRosout(msg) {
    if (!msg.msg || !LOG_NODE_PATTERN.test(msg.name || '')) return;
    if (msg.level < 20) return;
    addLog(msg.name, msg.msg, LOG_LEVELS[msg.level] || 'info');

    if (msg.msg.includes('Map saved successfully')) {
        mapSavedThisSession = true;
        setPhase('MAP SAVED - READY FOR ORCHESTRATION', 'ok');
    } else if (/Published (\d+) rack poses/.test(msg.msg)) {
        const n = parseInt(msg.msg.match(/Published (\d+) rack poses/)[1]);
        setPhase(`${n} RACKS DETECTED`, n > 0 ? 'active' : 'warn');
    } else if (msg.msg.includes('Popped a rack. Remaining:')) {
        onRackPopped(msg.msg);
    } else if (msg.msg.includes('Mission fully complete')) {
        setPhase('MISSION COMPLETE - DOCKED', 'ok');
    } else if (msg.msg.includes('E-STOP TRIGGERED')) {
        setPhase('E-STOP ENGAGED', 'danger');
    } else if (msg.msg.includes('E-Stop complete')) {
        setPhase('E-STOPPED - ROBOT IDLE', 'danger');
    }
}

const EXPLORE_PHASES = {
    exploration_started: ['EXPLORING', 'active'],
    exploration_in_progress: ['EXPLORING', 'active'],
    exploration_paused: ['EXPLORATION PAUSED', 'warn'],
    exploration_complete: ['EXPLORATION COMPLETE', 'active'],
    returning_to_origin: ['RETURNING TO ORIGIN', 'active'],
    returned_to_origin: ['MAPPING DONE - SAVING MAP', 'active'],
};

function subscribeExploreStatus() {
    subscribe('/explore/status', 'explore_lite_msgs/msg/ExploreStatus', (msg) => {
        const phase = EXPLORE_PHASES[msg.status];
        if (phase) setPhase(phase[0], phase[1]);
    });
}

// ---------------------------------------------------------------- Battery
function subscribeBattery() {
    subscribe('/battery_level', 'sensor_msgs/msg/BatteryState', (msg) => {
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
}

// ---------------------------------------------------------------- 2D map (custom canvas renderer)
const ctx = mapCanvas.getContext('2d');
const offscreenCanvas = document.createElement('canvas');
const offscreenCtx = offscreenCanvas.getContext('2d');
let currentMapInfo = null;
let currentMapScale = 1;
let lastRobotPose = null;

function subscribeMap() {
    subscribe('/map', 'nav_msgs/msg/OccupancyGrid', (msg) => {
        currentMapInfo = msg.info;
        const width = msg.info.width;
        const height = msg.info.height;

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

        offscreenCtx.putImageData(imgData, 0, 0);
        fitMapToContainer();
    });
}

function fitMapToContainer() {
    if (!currentMapInfo) return;
    const { width, height } = currentMapInfo;
    currentMapScale = Math.min(mapContainer.clientWidth / width, mapContainer.clientHeight / height) * 0.9;
    mapCanvas.width = width * currentMapScale;
    mapCanvas.height = height * currentMapScale;
    redrawMap();
}

// World (map frame) coordinates -> canvas pixels, in the flipped drawing space below.
function worldToCanvas(x, y) {
    const res = currentMapInfo.resolution;
    return [
        ((x - currentMapInfo.origin.position.x) / res) * currentMapScale,
        ((y - currentMapInfo.origin.position.y) / res) * currentMapScale,
    ];
}

function redrawMap() {
    if (!currentMapInfo) return;

    // Flip vertically (ROS map origin is bottom left, canvas is top left)
    ctx.save();
    ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
    ctx.scale(1, -1);
    ctx.translate(0, -mapCanvas.height);

    ctx.imageSmoothingEnabled = false; // Keep the pixelated sci-fi look
    ctx.drawImage(offscreenCanvas, 0, 0, mapCanvas.width, mapCanvas.height);

    drawRackMarkers();
    if (lastRobotPose) drawRobot(lastRobotPose);

    ctx.restore();
}

function drawRackMarkers() {
    globalRacks.forEach((rack, i) => {
        const [px, py] = worldToCanvas(rack.x, rack.y);
        const color = rack.checked ? '#39ff14' : '#ff5500';
        ctx.save();
        ctx.translate(px, py);
        // Diamond waypoint marker
        ctx.beginPath();
        ctx.moveTo(0, 7); ctx.lineTo(7, 0); ctx.lineTo(0, -7); ctx.lineTo(-7, 0);
        ctx.closePath();
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.shadowBlur = 8;
        ctx.shadowColor = color;
        ctx.stroke();
        if (rack.checked) { ctx.fillStyle = 'rgba(57, 255, 20, 0.35)'; ctx.fill(); }
        // Label (un-flip the y axis so the text is upright)
        ctx.scale(1, -1);
        ctx.shadowBlur = 0;
        ctx.fillStyle = color;
        ctx.font = '10px "Roboto Mono", monospace';
        ctx.fillText(String(i + 1), 9, 4);
        ctx.restore();
    });
}

function drawRobot(robotPose) {
    const [px, py] = worldToCanvas(robotPose.x, robotPose.y);
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(robotPose.yaw);

    // Sci-Fi Cyan Triangle for the robot
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
    ctx.restore();
}

// Live robot position. Odometry arrives at ~50 Hz; 10 Hz is plenty for the display.
function subscribeOdom() {
    subscribe('/odom', 'nav_msgs/msg/Odometry', (msg) => {
        // Convert quaternion to Euler yaw
        const q = msg.pose.pose.orientation;
        const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);

        lastRobotPose = {
            x: msg.pose.pose.position.x,
            y: msg.pose.pose.position.y,
            yaw: Math.atan2(siny_cosp, cosy_cosp)
        };
        redrawMap();
    }, { throttle_rate: 100 });
}

// ---------------------------------------------------------------- Racks
let globalRacks = [];

function onRackPoses(msg) {
    const racks = msg.poses.map((pose, index) => ({
        id: `RACK-${(index + 1).toString().padStart(3, '0')}`,
        x: pose.position.x,
        y: pose.position.y,
        location: `X: ${pose.position.x.toFixed(2)}, Y: ${pose.position.y.toFixed(2)}`,
        checked: false
    }));
    // The latched message is re-delivered after a reconnect: keep progress if it's
    // the same rack set, start over if it's a new mission's detections.
    const same = racks.length === globalRacks.length &&
        racks.every((r, i) => r.location === globalRacks[i].location);
    if (same) return;
    globalRacks = racks;
    renderRacks();
    redrawMap();
}

function onRackPopped(text) {
    const match = text.match(/Remaining:\s*(\d+)/);
    if (!match || globalRacks.length === 0) return;
    const remaining = parseInt(match[1]);
    const completedCount = globalRacks.length - remaining;

    // Mark the first 'completedCount' racks as checked
    globalRacks.forEach((rack, i) => { rack.checked = i < completedCount; });
    setPhase(remaining > 0 ? `VISITING RACKS ${completedCount}/${globalRacks.length}`
                           : 'ALL RACKS VISITED - RETURNING TO DOCK', 'active');
    renderRacks();
    redrawMap();
}

function renderRacks() {
    rackListContainer.innerHTML = '';
    if (globalRacks.length === 0) {
        rackSummary.innerText = '> AWAITING RACK DETECTION...';
        const li = document.createElement('li');
        li.className = 'rack-item';
        li.innerHTML = `
            <div>
                <div class="rack-id">AWAITING_DATA</div>
                <div class="rack-location">Scanning map for targets...</div>
            </div>
            <i class="fa-solid fa-spinner fa-spin rack-status-icon"></i>`;
        rackListContainer.appendChild(li);
        return;
    }

    const done = globalRacks.filter(r => r.checked).length;
    rackSummary.innerText = done === globalRacks.length
        ? `> ALL ${done} RACKS VERIFIED`
        : `> ${done}/${globalRacks.length} RACKS VERIFIED`;

    globalRacks.forEach(rack => {
        const li = document.createElement('li');
        li.className = `rack-item ${rack.checked ? 'checked' : ''}`;
        li.innerHTML = `
            <div>
                <div class="rack-id">${rack.id}</div>
                <div class="rack-location">${rack.location}</div>
            </div>
            <i class="fa-solid ${rack.checked ? 'fa-check-double' : 'fa-spinner fa-spin'} rack-status-icon"></i>`;
        rackListContainer.appendChild(li);
    });
}

// ---------------------------------------------------------------- UI (set up ONCE)
function initUI() {
    // View Navigation
    navButtons.forEach(btn => {
        btn.addEventListener('click', () => {
            navButtons.forEach(b => b.classList.remove('active'));
            viewSections.forEach(s => { s.classList.remove('active'); s.classList.add('hidden'); });

            btn.classList.add('active');
            const target = document.getElementById(btn.getAttribute('data-target'));
            target.classList.remove('hidden');
            target.classList.add('active');
            if (btn.getAttribute('data-target') === 'dashboard-view') fitMapToContainer();
        });
    });

    initJoystick();

    document.getElementById('btn-start-mapping').addEventListener('click', () => {
        if (publish('/gui/trigger_mapping', 'std_msgs/msg/Empty', {})) {
            mapSavedThisSession = false;
            setPhase('STARTING MISSION', 'active');
            addLog('gui', 'INIT_MAPPING sent.');
        }
    });

    document.getElementById('btn-start-orchestration').addEventListener('click', () => {
        // map_operation_node reads whatever map file is on disk right away. Before
        // this mission's map is saved that's the previous run's map.
        if (!mapSavedThisSession && !window.confirm(
            'No map has been saved in this session yet.\n\n' +
            'Rack detection will use the LAST SAVED map file. Continue anyway?')) {
            return;
        }
        if (publish('/gui/trigger_orchestration', 'std_msgs/msg/Empty', {})) {
            addLog('gui', 'EXEC_ORCHESTRATION sent.');
        }
    });

    document.getElementById('btn-estop').addEventListener('click', () => {
        stopJoystickStream();
        publish('/cmd_vel', 'geometry_msgs/msg/Twist', ZERO_TWIST);
        if (publish('/gui/estop', 'std_msgs/msg/Empty', {})) {
            setPhase('E-STOP ENGAGED', 'danger');
            addLog('gui', 'E-STOP sent (zero velocity + stop mission).', 'error');
        }
        console.log("E-STOP triggered (0-velocity and process kill signal sent).");
    });

    window.addEventListener('resize', fitMapToContainer);

    setButtonsEnabled(false);
    setPhase('IDLE', 'idle');
    renderRacks();
}

// ---------------------------------------------------------------- Teleoperation joystick
const ZERO_TWIST = { linear: { x: 0.0, y: 0.0, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: 0.0 } };
let joystickTwist = null;
let joystickTimer = null;

// nipplejs only fires 'move' while the stick moves, so a stick held still would
// send nothing. Stream the current command at 10 Hz while it is held instead.
function stopJoystickStream() {
    if (joystickTimer) clearInterval(joystickTimer);
    joystickTimer = null;
    joystickTwist = null;
}

function initJoystick() {
    const manager = nipplejs.create({
        zone: document.getElementById('joystick-zone'),
        mode: 'static',
        position: { left: '50%', top: '50%' },
        color: '#ff5500',
        size: 150
    });

    manager.on('move', (event, data) => {
        const max_linear = 0.5; // m/s
        const max_angular = 1.0; // rad/s
        const max_distance = 75.0; // max size of joystick

        joystickTwist = {
            linear: { x: Math.sin(data.angle.radian) * max_linear * data.distance / max_distance, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: -Math.cos(data.angle.radian) * max_angular * data.distance / max_distance }
        };
        if (!joystickTimer) {
            publish('/cmd_vel', 'geometry_msgs/msg/Twist', joystickTwist, true);
            joystickTimer = setInterval(() => {
                if (joystickTwist) publish('/cmd_vel', 'geometry_msgs/msg/Twist', joystickTwist, true);
            }, 100);
        }
    });

    manager.on('end', () => {
        // Stop robot when joystick released
        stopJoystickStream();
        publish('/cmd_vel', 'geometry_msgs/msg/Twist', ZERO_TWIST, true);
    });
}

// Start connection attempt on load
window.onload = () => {
    initUI();
    connectToROS();
};
