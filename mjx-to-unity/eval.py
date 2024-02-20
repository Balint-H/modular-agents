import onnxruntime

print("Importing...")
import numpy as np
from brax.training.acme import running_statistics

np.set_printoptions(precision=3, suppress=True, linewidth=100)

import jax
from jax import numpy as jp
import numpy as np

from brax import envs

from brax.training.agents.ppo import networks as ppo_networks
from brax.io import model
import mujoco
from mujoco import mjx
from video import show_video
import jax.numpy as jnp
from humanoid import Humanoid
from brax.io import html, mjcf, model

print("register env...")
envs.register_environment('humanoid', Humanoid)

# instantiate the environment
env_name = 'humanoid'
env = envs.get_environment(env_name)

eval_env = envs.get_environment(env_name)

rng = jax.random.PRNGKey(0)
n_steps = 1000
render_every = 2

# Load the ONNX model
onnx_model_path = 'feedforward_model.onnx'
onnx_session = onnxruntime.InferenceSession(onnx_model_path)
# Run inference
input_name = onnx_session.get_inputs()[0].name
output_names = [output.name for output in onnx_session.get_outputs()]

"""# MJX Policy in MuJoCo

We can also perform the physics step using the original MuJoCo python bindings to show that the policy trained in MJX works in MuJoCo.
"""

mj_model = eval_env.sys.mj_model
mj_data = mujoco.MjData(mj_model)
mujoco.mj_resetData(mj_model, mj_data)

renderer = mujoco.Renderer(mj_model)
ctrl = jp.zeros(mj_model.nu)
eval_env._n_frames = 1
images = []
for i in range(n_steps):
  obs = eval_env._get_obs(mjx.put_data(mj_model, mj_data), ctrl)
  onnx_result = onnx_session.run(output_names, {input_name: np.array([obs])})
  onnx_ctrl = onnx_result[1][0]

  mj_data.ctrl = onnx_ctrl
  for _ in range(eval_env._n_frames):
    mujoco.mj_step(mj_model, mj_data)  # Physics step using MuJoCo mj_step.

  if i % render_every == 0:
    renderer.update_scene(mj_data, camera='side')
    images.append(renderer.render())

show_video(images, fps=1.0 / eval_env.dt / render_every, )
