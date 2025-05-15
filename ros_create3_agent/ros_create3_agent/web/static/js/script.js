document.addEventListener('DOMContentLoaded', function () {
    // DOM element references
    const chatMessages = document.getElementById('chat-messages');
    const messageInput = document.getElementById('message-input');
    const sendButton = document.getElementById('send-button');
    const batteryLevel = document.getElementById('battery-level');
    const dockStatus = document.getElementById('dock-status');
    const hazardsList = document.getElementById('hazards-list');
    const pickedUpStatus = document.getElementById('picked-up-status');
    const proximityReadings = document.getElementById('proximity-readings');
    const cliffReadings = document.getElementById('cliff-readings');
    const MAX_MESSAGES = 100;

    // Initialize markdown-it for agent message formatting
    const md = window.markdownit({ html: false, linkify: true, typographer: true });

    // Initial load and polling for chat and robot status
    fetchChatHistoryAndRobotStatus();
    setInterval(fetchChatHistoryAndRobotStatus, 500); // Poll every 0.5 seconds

    // Event listeners for sending messages
    sendButton.addEventListener('click', sendMessage);
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
        batteryLevel.textContent = status.battery_level;
        dockStatus.textContent = status.dock_status;
        hazardsList.innerHTML = '';
        // Show hazards if present
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
        // Picked up status
        pickedUpStatus.textContent = status.is_picked_up ? 'Yes' : 'No';
        pickedUpStatus.className = status.is_picked_up ? 'picked-up-true' : 'picked-up-false';
        // Update sensor readings
        updateSensorReadings(proximityReadings, status.ir_intensities, false);
        updateSensorReadings(cliffReadings, status.cliff_intensities, true);
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
    function sendMessage() {
        const message = messageInput.value.trim();
        if (!message) return;
        // Disable input while waiting for server response
        messageInput.disabled = true;
        sendButton.disabled = true;
        sendButton.classList.add('disabled');
        messageInput.value = '';
        fetch('/api/chat', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ message })
        })
            .then(response => response.json())
            .then(data => {
                updateChatHistory(data.history);
                updateRobotStatus(data.status);
                // Re-enable input after response
                messageInput.disabled = false;
                sendButton.disabled = false;
                sendButton.classList.remove('disabled');
                messageInput.focus();
            })
            .catch(error => {
                console.error('Error sending message:', error);
                messageInput.disabled = false;
                sendButton.disabled = false;
                sendButton.classList.remove('disabled');
            });
    }
});