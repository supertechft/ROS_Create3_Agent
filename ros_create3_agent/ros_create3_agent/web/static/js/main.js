// main.js - Entry point for the web app
import { fetchChatHistoryAndRobotStatus, sendMessage } from './modules/api.js';
import { setupUIControls, initializeMarkdown } from './modules/ui.js';

// Constants
const MAX_MESSAGES = 100;
const POLL_RATE_MS = 500;

// Global state for command execution tracking
let commandExecuting = false;
let statusCheckInterval = null;


// Export as globals for modules to access
window.appState = {
  MAX_MESSAGES,
  POLL_RATE_MS,
  commandExecuting,
  statusCheckInterval,
};


document.addEventListener('DOMContentLoaded', function () {
  // Initialize UI elements and get references
  const elements = setupUIControls();
  const md = initializeMarkdown();
  window.appElements = elements;
  window.md = md;


  // Event listeners for sending messages
  elements.sendButton.addEventListener('click', () => sendMessage(false));
  elements.audioButton.addEventListener('click', () => sendMessage(true));
  elements.messageInput.addEventListener('keypress', (e) => {
    if (e.key === 'Enter') sendMessage(false);
  });


  // Initial load and polling for chat and robot status
  fetchChatHistoryAndRobotStatus();
  setInterval(fetchChatHistoryAndRobotStatus, POLL_RATE_MS);
});
