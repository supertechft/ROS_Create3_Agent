import threading
import webbrowser
import os
import time
from typing import List, Dict

# Flask imports
from flask import (
    Flask,
    render_template,
    request,
    jsonify,
)

# ROS imports
from rclpy.node import Node

# Internal imports
from ros_create3_agent.robot.robot_state import get_robot_state
from ros_create3_agent.logging import get_logger
from ros_create3_agent.rosa_config import WEB_PORT, MAX_CHAT_HISTORY
from ros_create3_agent.utils.ros_threading import run_in_executor

# Get a logger for this module
logger = get_logger(__name__)

# Global variables
app = Flask(
    __name__,
    template_folder=os.path.join(os.path.dirname(__file__), "templates"),
    static_folder=os.path.join(os.path.dirname(__file__), "static"),
)

# Set in initialize()
rosa = None
robot_state = None
audio_system = None

# Chat history (list of dicts: {content, sender (user/robot/agent)})
chat_history = []

# Track command execution
command_executing = False
command_executing_lock = threading.Lock()


# Helper to trim chat history
def _trim_chat_history():
    global chat_history
    if len(chat_history) > MAX_CHAT_HISTORY:
        chat_history = chat_history[-MAX_CHAT_HISTORY:]


def _add_user_message(content: str):
    chat_history.append({"content": content, "sender": "user"})
    _trim_chat_history()


def add_robot_message(content: str):
    chat_history.append({"content": content, "sender": "robot"})
    _trim_chat_history()

    # Play audio for the message if it matches a priority pattern
    try:
        from ros_create3_agent.web.audio.audio_system import play_audio_for_message

        play_audio_for_message(content)
    except Exception as e:
        # Fail silently - don't let audio errors affect robot functionality
        logger.error(f"Audio playback error: {e}")


def _add_agent_message(content: str):
    chat_history.append({"content": content, "sender": "agent"})
    _trim_chat_history()


def initialize(node: Node, ros_agent) -> None:
    """Initialize the web interface for the Create 3 robot agent."""
    global rosa, robot_state, audio_system
    rosa = ros_agent
    robot_state = get_robot_state(node)

    # Initialize audio system
    try:
        from ros_create3_agent.web.audio.audio_system import get_audio_system

        audio_system = get_audio_system()
        logger.info("Audio system initialized for robot messages")
    except Exception as e:
        logger.error(f"Failed to initialize audio system: {e}")

    # Initialize chat with welcome message
    welcome_msg = "Welcome to the Create 3 Robot Assistant. How can I help you today?"
    _add_agent_message(welcome_msg)

    # Start Flask server in background thread
    web_thread = threading.Thread(target=_run_flask_app, daemon=True)
    web_thread.start()

    # Give Flask time to start before opening browser
    time.sleep(2)
    webbrowser.open(f"http://localhost:{WEB_PORT}")
    logger.info(f"Web interface initialized on http://localhost:{WEB_PORT}")


def _get_chat_history() -> List[Dict[str, str]]:
    """Return our own chat history for the web UI."""
    return list(chat_history)


def _run_flask_app() -> None:
    """Run the Flask app in a separate thread."""
    app.run(host="0.0.0.0", port=WEB_PORT, threaded=True, debug=False)


def _process_user_input(user_input: str) -> None:
    """
    This function adds the message to chat history and offloads LLM executing to a background thread.
    """
    logger.info(f"Executing command via web: {user_input}")
    # Using shared thread pool to prevent blocking the Flask thread
    run_in_executor(_process_user_input_background, user_input)


def _is_executing_command():
    """Check if a command is currently being executed."""
    with command_executing_lock:
        return command_executing


def _set_command_executing(state):
    """Set the command executing state."""
    global command_executing
    with command_executing_lock:
        command_executing = state


def _process_user_input_background(user_input: str):
    """
    Background worker that processes user input through ROSA.
    Handles both text and audio input and adds responses to chat history.
    This runs in a separate thread to prevent blocking the Flask server.
    """
    try:
        _set_command_executing(True)

        # Check if the user input is an audio command
        if user_input.lower().strip() == "audio":
            logger.info("Listening for verbal input...")
            _add_agent_message("Listening...")

            # Get audio transcription from Rosa
            transcription = rosa.listen()
            if transcription:
                logger.info(f"Transcribed audio: {transcription}")
                _add_user_message(f"🎤 {transcription}")
                response = rosa.invoke(transcription)
            else:
                logger.warning("No speech detected or transcription failed")
                response = "I couldn't hear anything or understand what was said. Please try again."
        else:
            # Regular text input processing - this can take time due to LLM API calls
            _add_user_message(user_input)  # Add user message to chat history
            response = rosa.invoke(user_input)

        _add_agent_message(response)
    except Exception as e:
        error_msg = f"Error executing command: {str(e)}"
        logger.error(error_msg)
        _add_agent_message(error_msg)
    finally:
        _set_command_executing(False)


@app.route("/")
def index():
    """Render the main page."""
    return render_template("index.html", max_chat_history=MAX_CHAT_HISTORY)


@app.route("/api/chat", methods=["GET"])
def get_chat():
    """API endpoint to get chat history."""
    return jsonify({"history": _get_chat_history(), "status": robot_state.get_state()})


@app.route("/api/chat", methods=["POST"])
def post_message():
    """API endpoint to post a new message."""
    if not request.json or "message" not in request.json:
        return jsonify({"error": "Invalid request"}), 400

    user_input = request.json["message"]
    _process_user_input(user_input)

    return jsonify(
        {
            "history": _get_chat_history(),
            "status": robot_state.get_state(),
        }
    )


@app.route("/api/status", methods=["GET"])
def api_status():
    """API endpoint to check if a command is being executed."""
    return jsonify({"executing": _is_executing_command()})
