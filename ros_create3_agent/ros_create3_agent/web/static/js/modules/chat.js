// chat.js - Handle chat messages and UI updates

/**
 * Update the chat UI with the latest messages
 * @param {Array} history - Array of chat messages
 */
export function updateChatHistory(history) {
  const chatMessages = window.appElements.chatMessages;
  const MAX_MESSAGES = window.appState.MAX_MESSAGES;

  // Preserve scroll position if user is at the bottom
  const wasAtBottom = chatMessages.scrollHeight - chatMessages.clientHeight <= chatMessages.scrollTop + 10;
  chatMessages.innerHTML = '';

  // Only show the most recent MAX_MESSAGES
  const messagesToShow = history.length > MAX_MESSAGES ? history.slice(-MAX_MESSAGES) : history;
  messagesToShow.forEach(addMessageToUI);

  if (wasAtBottom) chatMessages.scrollTop = chatMessages.scrollHeight;
}


/**
 * Add a single message to the chat UI
 * @param {Object} message - Message object with sender and content
 */
function addMessageToUI(message) {
  const chatMessages = window.appElements.chatMessages;
  const md = window.md;

  const messageElement = document.createElement('div');
  messageElement.className = `message ${message.sender}`;

  // Render agent messages as markdown, others as plain text
  messageElement.innerHTML = (message.sender === 'agent') ? md.render(message.content) : message.content;
  chatMessages.appendChild(messageElement);
}
