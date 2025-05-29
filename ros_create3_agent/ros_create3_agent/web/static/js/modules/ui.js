// ui.js - Handle UI elements and state

/**
 * Setup UI controls and return references
 * @returns {Object} Object containing references to UI elements
 */
export function setupUIControls() {
  return {
    chatMessages: document.getElementById('chat-messages'),
    messageInput: document.getElementById('message-input'),
    sendButton: document.getElementById('send-button'),
    audioButton: document.getElementById('audio-button'),
    batteryLevel: document.getElementById('battery-level'),
    dockStatus: document.getElementById('dock-status'),
    hazardsList: document.getElementById('hazards-list'),
    pickedUpStatus: document.getElementById('picked-up-status'),
    odometryStatus: document.getElementById('odometry-status'),
    imuStatus: document.getElementById('imu-status'),
    stopStatus: document.getElementById('stop-status'),
    proximityReadings: document.getElementById('proximity-readings'),
    cliffReadings: document.getElementById('cliff-readings'),
  };
}


/**
 * Initialize markdown-it for agent message formatting
 * @returns {Object} Markdown-it instance
 */
export function initializeMarkdown() {
  return window.markdownit({ html: false, linkify: true, typographer: true });
}


/**
 * Disable UI elements during command execution
 */
export function disableUIElements() {
  const elements = window.appElements;
  elements.messageInput.disabled = true;
  elements.audioButton.disabled = true;
  elements.audioButton.classList.add('disabled');
  elements.sendButton.disabled = true;
  elements.sendButton.classList.add('disabled');
}


/**
 * Enable UI elements after command execution
 */
export function enableUIElements() {
  const elements = window.appElements;
  elements.messageInput.disabled = false;
  elements.audioButton.disabled = false;
  elements.audioButton.classList.remove('disabled');
  elements.sendButton.disabled = false;
  elements.sendButton.classList.remove('disabled');
  elements.messageInput.focus();
}
