from rosa import ROSA


"""
import subprocess
subprocess.Popen(
  ["ros2", "launch", "irobot_create_gz_bringup", "create3_gz.launch.py"]
)
"""


rosa = ROSA(
  ros_version=2,
  llm=get_your_llm(),
  tools=[drive],
  prompts=prompts
)
rosa.invoke("Drive forward 1 meter.")
