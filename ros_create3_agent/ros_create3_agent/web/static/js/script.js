document.addEventListener('DOMContentLoaded', function () {
    // DOM element references
    const chatMessages = document.getElementById('chat-messages');
    const messageInput = document.getElementById('message-input');
    const sendButton = document.getElementById('send-button');
    const audioButton = document.getElementById('audio-button');
    const batteryLevel = document.getElementById('battery-level');
    const dockStatus = document.getElementById('dock-status');
    const hazardsList = document.getElementById('hazards-list');
    const pickedUpStatus = document.getElementById('picked-up-status');
    const odometryStatus = document.getElementById('odometry-status');
    const imuStatus = document.getElementById('imu-status');
    const stopStatus = document.getElementById('stop-status');
    const proximityReadings = document.getElementById('proximity-readings');
    const cliffReadings = document.getElementById('cliff-readings');
    const MAX_MESSAGES = 100;
    const POLL_RATE_MS = 500;

    // Track command execution state
    let commandExecuting = false;
    let statusCheckInterval = null;


    // Initialize markdown-it for agent message formatting
    const md = window.markdownit({ html: false, linkify: true, typographer: true });


    // Initial load and polling for chat and robot status
    fetchChatHistoryAndRobotStatus();
    setInterval(fetchChatHistoryAndRobotStatus, POLL_RATE_MS);


    // Event listeners for sending messages
    sendButton.addEventListener('click', sendMessage);
    audioButton.addEventListener('click', function () {
        sendMessage(true);
    });
    messageInput.addEventListener('keypress', function (e) {
        if (e.key === 'Enter') sendMessage();
    });


    // Fetch chat history and robot status from the server
    function fetchChatHistoryAndRobotStatus() {
        fetch('/api/chat')
            .then(response => response.json())
            .then(data => {
                updateChatHistory(data.history);
                updateRobotStatus(data.status);
            })
            .catch(error => console.error('Error fetching chat history:', error));
    }

    // Check command execution status
    function checkCommandStatus() {
        fetch('/api/status')
            .then(response => response.json())
            .then(data => {
                if (data.executing !== commandExecuting) {
                    commandExecuting = data.executing;
                    if (commandExecuting) {
                        disableUIElements();
                    } else {
                        enableUIElements();
                        if (statusCheckInterval) {
                            clearInterval(statusCheckInterval);
                            statusCheckInterval = null;
                        }
                    }
                }
            })
            .catch(error => console.error('Error checking command status:', error));
    }

    // Helper functions to disable/enable UI
    function disableUIElements() {
        messageInput.disabled = true;
        audioButton.disabled = true;
        audioButton.classList.add('disabled');
        sendButton.disabled = true;
        sendButton.classList.add('disabled');
    }

    function enableUIElements() {
        messageInput.disabled = false;
        audioButton.disabled = false;
        audioButton.classList.remove('disabled');
        sendButton.disabled = false;
        sendButton.classList.remove('disabled');
        messageInput.focus();
    }


    // Update the chat UI with the latest messages
    function updateChatHistory(history) {
        // Preserve scroll position if user is at the bottom
        const wasAtBottom = chatMessages.scrollHeight - chatMessages.clientHeight <= chatMessages.scrollTop + 10;
        chatMessages.innerHTML = '';
        // Only show the most recent MAX_MESSAGES
        const messagesToShow = history.length > MAX_MESSAGES ? history.slice(-MAX_MESSAGES) : history;
        messagesToShow.forEach(addMessageToUI);
        if (wasAtBottom) chatMessages.scrollTop = chatMessages.scrollHeight;
    }


    // Add a single message to the chat UI
    function addMessageToUI(message) {
        const messageElement = document.createElement('div');
        messageElement.className = `message ${message.sender}`;
        // Render agent messages as markdown, others as plain text
        messageElement.innerHTML = (message.sender === 'agent') ? md.render(message.content) : message.content;
        chatMessages.appendChild(messageElement);
    }

    // Update robot status UI (battery, dock, hazards, sensors)
    function updateRobotStatus(status) {
        // Update battery information with status
        const battery = status.battery || {};
        batteryLevel.textContent = battery.level || "Unknown";

        const batteryStatus = document.getElementById('battery-status');
        const status_text = battery.status || "Unknown";
        batteryStatus.textContent = status_text;
        batteryStatus.className = 'battery-status';
        batteryStatus.classList.add('battery-status-' + status_text);

        // Update battery voltage and current (2 decimal places)
        const batteryVoltage = document.getElementById('battery-voltage');
        const batteryCurrent = document.getElementById('battery-current');
        const voltage = battery.voltage !== null ? battery.voltage.toFixed(2) : "0.00";
        const current = battery.current !== null ? battery.current.toFixed(2) : "0.00";
        batteryVoltage.textContent = `${voltage} V`;
        batteryCurrent.textContent = `${current} A`;


        // Update dock status
        updateDockStatus(status.dock_status);


        // Update hazards list
        hazardsList.innerHTML = '';
        if (status.hazards && status.hazards.length > 0) {
            status.hazards.forEach(hazard => {
                const hazardElement = document.createElement('div');
                // Classify hazard type for styling
                let hazardType = 'unknown';
                if (hazard.type.includes('BUMP')) hazardType = 'bump';
                else if (hazard.type.includes('CLIFF')) hazardType = 'cliff';
                else if (hazard.type.includes('WHEEL_DROP')) hazardType = 'wheel-drop';
                else if (hazard.type.includes('BACKUP_LIMIT')) hazardType = 'backup-limit';
                else if (hazard.type.includes('OBJECT_PROXIMITY')) hazardType = 'proximity';
                hazardElement.className = `hazard-item hazard-${hazardType}`;
                hazardElement.textContent = hazard.description;

                // Add location if available
                if (hazard.location) {
                    const locationSpan = document.createElement('span');
                    locationSpan.className = 'hazard-location';
                    locationSpan.textContent = ` (${hazard.location})`;
                    hazardElement.appendChild(locationSpan);
                }
                hazardsList.appendChild(hazardElement);
            });
        } else {
            hazardsList.textContent = 'None';
        }


        // Update picked up status
        if (status.is_picked_up === null) {
            pickedUpStatus.textContent = 'Unknown';
            pickedUpStatus.className = '';
        } else {
            pickedUpStatus.textContent = status.is_picked_up ? 'Yes' : 'No';
            pickedUpStatus.className = status.is_picked_up ? 'picked-up-true' : 'picked-up-false';
        }


        // Update odometry status
        updateOdometryStatus(odometryStatus, status.odometry);


        // Update IMU status
        updateImuStatus(imuStatus, status.imu);


        // Update stop status
        updateStopStatus(stopStatus, status.stop_status);


        // Update sensor readings
        updateSensorReadings(proximityReadings, status.ir_intensities, false);
        updateSensorReadings(cliffReadings, status.cliff_intensities, true);
    }


    // Update dock status display
    function updateDockStatus(dockStatusInfo) {
        const dockStatusElement = document.getElementById('dock-status');
        const dockVisibleElement = document.getElementById('dock-visible');

        const dockData = dockStatusInfo || {};
        const isDocked = dockData.is_docked;
        const isVisible = dockData.dock_visible;

        // Update docked status (Yes/No like picked-up-status)
        if (isDocked === null || isDocked === undefined) {
            dockStatusElement.textContent = 'Unknown';
            dockStatusElement.className = 'status-value';
        } else if (isDocked) {
            dockStatusElement.textContent = 'Yes';
            dockStatusElement.className = 'status-value picked-up-true';
        } else {
            dockStatusElement.textContent = 'No';
            dockStatusElement.className = 'status-value picked-up-false';
        }

        // Update station visible status (Yes/No)
        if (isVisible === null || isVisible === undefined) {
            dockVisibleElement.textContent = 'Unknown';
            dockVisibleElement.className = 'status-value';
        } else if (isVisible) {
            dockVisibleElement.textContent = 'Yes';
            dockVisibleElement.className = 'status-value picked-up-true';
        } else {
            dockVisibleElement.textContent = 'No';
            dockVisibleElement.className = 'status-value picked-up-false';
        }
    }

    // Update odometry status display
    function updateOdometryStatus(container, odometry) {
        container.innerHTML = '';
        if (odometry && Object.keys(odometry).length > 0) {
            // Position
            const positionDiv = document.createElement('div');
            positionDiv.className = 'odom-position';
            const position = odometry.position || {};
            positionDiv.textContent = `Position: x=${(position.x || 0).toFixed(2)}m, y=${(position.y || 0).toFixed(2)}m`;

            // Orientation (quaternion)
            const orientationDiv = document.createElement('div');
            orientationDiv.className = 'odom-orientation';
            const orientation = odometry.orientation || {};
            orientationDiv.textContent = `Orientation: [${(orientation.x || 0).toFixed(2)}, ${(orientation.y || 0).toFixed(2)}, ${(orientation.z || 0).toFixed(2)}, ${(orientation.w || 0).toFixed(2)}]`;

            // Velocity
            const velocityDiv = document.createElement('div');
            velocityDiv.className = 'odom-velocity';
            const linearVel = odometry.linear_velocity || {};
            const angularVel = odometry.angular_velocity || {};
            velocityDiv.textContent = `Speed: ${(linearVel.x || 0).toFixed(2)} m/s, Turn: ${(angularVel.z || 0).toFixed(2)} rad/s`;

            container.appendChild(positionDiv);
            container.appendChild(orientationDiv);
            container.appendChild(velocityDiv);
        } else {
            container.textContent = 'No data';
        }
    }

    // Update IMU status display
    function updateImuStatus(container, imu) {
        container.innerHTML = '';
        if (imu && Object.keys(imu).length > 0) {
            // Orientation (simplified)
            const orientationDiv = document.createElement('div');
            orientationDiv.className = 'imu-orientation';
            const orientation = imu.orientation || {};
            orientationDiv.textContent = `Orientation: [${(orientation.x || 0).toFixed(2)}, ${(orientation.y || 0).toFixed(2)}, ${(orientation.z || 0).toFixed(2)}, ${(orientation.w || 0).toFixed(2)}]`;

            // Linear acceleration
            const accelDiv = document.createElement('div');
            accelDiv.className = 'imu-accel';
            const accel = imu.linear_acceleration || {};
            accelDiv.textContent = `Accel: [${(accel.x || 0).toFixed(2)}, ${(accel.y || 0).toFixed(2)}, ${(accel.z || 0).toFixed(2)}] m/s²`;

            // Angular velocity
            const gyroDiv = document.createElement('div');
            gyroDiv.className = 'imu-gyro';
            const gyro = imu.angular_velocity || {};
            gyroDiv.textContent = `Gyro: [${(gyro.x || 0).toFixed(2)}, ${(gyro.y || 0).toFixed(2)}, ${(gyro.z || 0).toFixed(2)}] rad/s`;

            container.appendChild(orientationDiv);
            container.appendChild(accelDiv);
            container.appendChild(gyroDiv);
        } else {
            container.textContent = 'No data';
        }
    }

    // Update stop status display
    function updateStopStatus(container, stopStatus) {
        if (!stopStatus || stopStatus.is_stopped === null) {
            container.textContent = "Unknown";
            container.className = "status-value";
        } else if (stopStatus.is_stopped) {
            container.textContent = "Stopped";
            container.className = "status-value sensor-high";
        } else {
            container.textContent = "Moving";
            container.className = "status-value success-green";
        }
    }

    // Update proximity or cliff sensor readings
    function updateSensorReadings(container, readings, isCliff) {
        container.innerHTML = '';
        if (readings && Object.keys(readings).length > 0) {
            for (const [sensorName, value] of Object.entries(readings)) {
                const readingElement = document.createElement('div');
                readingElement.className = 'sensor-reading';
                // Color code based on value and sensor type
                if (isCliff) {
                    if (value < 20) readingElement.classList.add('sensor-reading-high');
                    else if (value < 50) readingElement.classList.add('sensor-reading-medium');
                    else readingElement.classList.add('sensor-reading-low');
                } else {
                    if (value > 50) readingElement.classList.add('sensor-reading-high');
                    else if (value > 20) readingElement.classList.add('sensor-reading-medium');
                    else readingElement.classList.add('sensor-reading-low');
                }
                readingElement.textContent = `${sensorName}: ${value}`;
                container.appendChild(readingElement);
            }
        } else {
            container.textContent = 'No data';
        }
    }


    // Send a user message to the server and update UI on response
    function sendMessage(audio = false) {
        const message = (audio) ? "audio" : messageInput.value.trim();
        if (!message) return;

        // Disable UI elements immediately
        disableUIElements();
        messageInput.value = '';

        // Start polling for command status using the defined poll rate
        if (!statusCheckInterval) {
            statusCheckInterval = setInterval(checkCommandStatus, POLL_RATE_MS);
        }

        // Send message to flask server
        fetch('/api/chat', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ message })
        })
            .then(response => response.json())
            .then(data => {
                updateChatHistory(data.history);
                updateRobotStatus(data.status);
            })
            .catch(error => {
                console.error('Error sending message:', error);
                // If there's an error in the API call itself,
                // re-enable the UI since the command never started executing
                if (statusCheckInterval) {
                    clearInterval(statusCheckInterval);
                    statusCheckInterval = null;
                }
                enableUIElements();
            });
    }
});