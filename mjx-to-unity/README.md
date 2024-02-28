# MJX to Unity conversion
The incldued python scripts form the start of a toolkit to convert a control policy trained using MJX to a Unity ML-Agents compatible format (ONNX).
All scripts were primarily tested with the policy from the [humanoid control example of MJX](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/mjx/tutorial.ipynb).


## `convert_to_onnx.py`:
This script takes as input a binary (e.g. mjx_brax_policy.pickle), reads the weight and bias information of the layers and constructs an ONNX graph layer by layer. Optionally we can perform a normalization step at the start in the graph. The architecture is fairly tied to the default PPO policy in Brax, the converter is not expected to work for any operation not present in that.


## `eval.py`:
Loads the scene with MJX, then runs inference using CPU MuJoCo using the ONNX policy.

## `test_inference.py`:
Runs a single observation vector through both Brax and ONNX policies for comparison (using deterministic inference). Expects you to perform the comparison by hand using debug tools, not an automated test.