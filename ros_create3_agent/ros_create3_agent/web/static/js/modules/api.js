// api.js - Handles all API communication

import { disableUIElements, enableUIElements } from './ui.js';
import { updateChatHistory } from './chat.js';
import { updateRobotStatus } from './robot/update.js';


/**
 * Fetch chat history and robot status
 */
export function fetchChatHistoryAndRobotStatus() {
  fetch('/api/chat')
    .then(response => response.json())
    .then(data => {
      updateChatHistory(data.history);
      updateRobotStatus(data.status);
    })
    .catch(error => console.error('Error fetching chat history:', error));
}


/**
 * Check command execution status
 */
export function checkCommandStatus() {
  fetch('/api/status')
    .then(response => response.json())
    .then(data => {
      if (data.executing !== window.appState.commandExecuting) {
        window.appState.commandExecuting = data.executing;
        if (window.appState.commandExecuting) {
          disableUIElements();
        } else {
          enableUIElements();
          if (window.appState.statusCheckInterval) {
            clearInterval(window.appState.statusCheckInterval);
            window.appState.statusCheckInterval = null;
          }
        }
      }
    })
    .catch(error => console.error('Error checking command status:', error));
}


/**
 * Sends a message to the server
 */
export function sendMessage(isAudio = false) {
  const { messageInput } = window.appElements;
  const message = (isAudio) ? "audio" : messageInput.value.trim();
  if (!message) return;


  // Disable UI elements immediately
  disableUIElements();
  messageInput.value = '';


  // Start polling for command status using the defined poll rate
  if (!window.appState.statusCheckInterval) {
    window.appState.statusCheckInterval = setInterval(checkCommandStatus, window.appState.POLL_RATE_MS);
  }


  // Send message
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
      if (window.appState.statusCheckInterval) {
        clearInterval(window.appState.statusCheckInterval);
        window.appState.statusCheckInterval = null;
      }
      enableUIElements();
    });
}
