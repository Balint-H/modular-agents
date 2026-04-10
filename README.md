# Modular Agents

![A humanoid walking, following a transparent reference animation](./UnityMjExamples/Recordings/drecon_walk.gif)
*Steady-state humanoid walking learned with DReCon.*

**Modular Agents** was started to investigate both interactive physics based character animation, and locomotion synthesis for applications in the field of assistive robotics[^1].  It demonstrates *Physics-based Humanoid Control using Reinforcement Learning*.  It has been implemented considering the following applications:

- Simulation of human biomechanics during locomotion.
- Physics-based character animation for games and interactive media.
- Neuromechanical simulations of motor control.

Examples of learning conditions/methods that have been re-implemented and trained using this toolkit include:

- Inverting a cart-pole pendulum.
- Level-ground walking with turns and stops using [DReCon](https://dl.acm.org/doi/abs/10.1145/3355089.3356536)[^2].
- Step ascension/descension with turns and stops using DReCon.
- Walking clip motion tracking with [DeepMimic](https://dl.acm.org/doi/abs/10.1145/3197517.3201311)[^3].
- Walking clip adversarial imitation with [AMP](https://dl.acm.org/doi/abs/10.1145/3450626.3459670)[^4].

The packages were developed primarily with the control of humanoid characters in mind. However, most systems would translate well to robotic or abstract contexts. The training environments use the [MuJoCo physics engine plugin](https://mujoco.readthedocs.io/en/latest/unity.html), with planned support for Unity's native PhysX. 

### Example projects

To get started with the package we provide two example projects:

- [**Unity Mj Examples**](UnityMjExamples/README.md): it contains several examples of physics-based agents set up and trained using MuJoCo. This includes the inverting cart-pole pendulum, and simple walking and throwing animations trained using the *DeepMimic* and *DReCon* architectures. There is also guidance for structuring your own environments and training policies.

- [**Unity PhysX Examples**](UnityPhysxExamples/README.md): We have worked in translating the MuJoCo environments to PhysX, with current challenges in working around the limited access to the PhysX runtime. The PhysX example project is not ready for release. Contributions welcome, get in touch if you are interested!

### Is this for me?

At the moment the package is more suited for researchers or users actively involved in the physics-based animation field. Applying the package to a new project is a hands-on process. 

The current way to construct a learning environment, is to create a new project, import the `modular-agents.core` plus other relevant packages (e.g. `modular-agents.mujoco` and `modular-agents.shared-assets`) of one of the previous example projects, copy a working training environment into the new project, and modify it to the specifications of your new environment.

We have some scripts that simplify the workflows to edit existing environments, they can be found in `modular-agents.mujoco`, in the folder *EditEnvironments*. We will eventually add automated workflows to create training environments from a simple animated skeleton, but these developments are not yet completed.  

### Why Unity and ML-Agents?

Unity provides a user-friendly and visual interface for constructing virtual scenes. It also comes with nice rendering, and a large body of compatible assets and customizable workflows. 
Environment logic is flexibile and performant to configure with C# scripting. Lastly, a large number of tools for handling human motion is available for Unity, which is convenient for generating reference motion.

ML-Agents separates environment and learning logic, making it very accessible to users new to Reinforcement Learning. Several *off-the shelf* policy types and learning algorithms are already implemented in it, that can be quickly configured, or extended with plug-ins.  ML-Agents also supports building Gym compatible environments, making it possible to pair built scenes with other learning frameworks.

Due to the diversity of locomotion environments, it is tempting to create new custom observations/rewards for each condition, or monolithic components that do too many things at once.  With Modular Agents we try to provide templates and scripts that let you reuse your/our code as much as possible, without the behaviours growing out of control.

### Further information

To understand the general package organisation of **Modular-Agents**, see the documentation found [here](modular-agents/README.md). For more information on the main components of **Modular-Agents**, which are *RL signals* and *Training events* , see the documentation of the [`core`](<modular-agents/core>) package.  The components specific to the simulation of humanoids are within the [`physx`](<modular-agents/physx>) and [`mujoco`](<modular-agents/mujoco>) packages, which have roughly matching content in the Unity's PhysX and  MuJoCo physics engines respectively. 

### Contributors

- [Balint Hodossy](https://github.com/Balint-H) (Imperial College London) 
- [Joan Llobera](https://joanllobera.github.io/) ([Artanim Foundation](https://artanim.ch/)) 

The work of  [Balint Hodossy](https://github.com/Balint-H) was partially supported by the Artanim Foundation, the UKRI CDT in Prosthetics and Orthotics (Grant No. EP/S02249X/1) and the [Natural BionicS initative](https://www.imperial.ac.uk/neuromechanics-rehabilitation-technology/research/active-projects/natural-bionics/) (Grant agreement ID: 810346). 

The work of [Joan Llobera](https://joanllobera.github.io/) was supported by the Artanim Foundation and the [Presence-XR](https://presence-xr.eu/)  EU project (Grant agreement ID: 101135025)

## References

  [^1]: Hodossy, B.K. and Farina, D., 2023. Shared Autonomy Locomotion Synthesis With a Virtual Powered Prosthetic Ankle. IEEE Transactions on Neural Systems and Rehabilitation Engineering, 31, pp.4738-4748. [DOI](https://doi.org/10.1109/tnsre.2023.3336713)
  [^2]: Bergamin, K., Clavet, S., Holden, D. and Forbes, J.R., 2019. DReCon: data-driven responsive control of physics-based characters. ACM Transactions On Graphics (TOG), 38(6), pp.1-11.
  [^3]: Peng, X.B., Abbeel, P., Levine, S. and Van de Panne, M., 2018. Deepmimic: Example-guided deep reinforcement learning of physics-based character skills. ACM Transactions On Graphics (TOG), 37(4), pp.1-14.
  [^4]: Peng, X.B., Ma, Z., Abbeel, P., Levine, S. and Kanazawa, A., 2021. Amp: Adversarial motion priors for stylized physics-based character control. ACM Transactions on Graphics (ToG), 40(4), pp.1-20.

### See also

This package was inspired by [Marathon Environments](https://joanllobera.github.io/marathon-envs/). This older PhysX package contain some previous benchmarks, together with a physics-based humanoid controller called *MarathonController*. The currently developed branch is migrating to use the Modular Agents scripts. In the future we'd like to use these packages to implement character animation pipelines in PhysX and publish them at this repository.
