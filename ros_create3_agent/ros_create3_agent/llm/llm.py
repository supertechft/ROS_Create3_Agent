"""
ROSA Documentation
- https://github.com/nasa-jpl/rosa/wiki/Model-Configuration
- https://github.com/nasa-jpl/rosa/wiki/Supported-Models
"""

import os
import typing
from dotenv import load_dotenv

# https://python.langchain.com/docs/integrations/chat/openai/
from langchain_openai import ChatOpenAI

# https://huggingface.co/docs/inference-providers/providers/hf-inference#automatic-speech-recognition
from huggingface_hub import InferenceClient

# Import our centralized logging configuration
from ros_create3_agent.logging import get_logger

# Get a logger for this module
logger = get_logger(__name__)

# Load environment variables from .env
load_dotenv()


# Get an environment variable, with optional default value
def get_env_variable(name: str, default: typing.Optional[str] = None) -> str:
    value = os.environ.get(name, default)
    if value is None or value == "":
        logger.warning(f"Environment variable {name} not set and no default provided")
    return value


# Get a HuggingFace Inference client
def get_HF_inference():
    api_key = get_env_variable("HF_API_KEY")
    if not api_key:
        logger.error("HuggingFace API key not set. Please configure your .env file with your API key.")
        raise ValueError("HF_API_KEY environment variable not set")

    logger.info("Using HuggingFace Inference API")
    client = InferenceClient(provider="hf-inference", token=api_key, headers={"Content-Type": "audio/wav"})
    return client


# Get an OpenAI LLM instance
def get_llm():
    # Get OpenAI API key
    openai_api_key = get_env_variable("OPENAI_API_KEY")

    if not openai_api_key:
        logger.error("OpenAI API key not set. Please configure your .env file with your API key.")
        raise ValueError("OPENAI_API_KEY environment variable not set")

    logger.info("Using OpenAI LLM")

    llm = ChatOpenAI(
        model_name="gpt-4o",
        temperature=0,
        max_tokens=None,
        timeout=None,
        max_retries=2,
        openai_api_key=openai_api_key,
    )
    return llm
