# Modular-Agents MuJoCo components

This package contains three assemblies:

- 'MujocoModularAgents' contains the main functionality of 'modular-agents' adapted to the specifics of MuJoCo. The scripts follow the same structure than in the [core modular agents](<../core/README.md>).
- SPD contains an implementation of a Stable Proportional-Derivative Controller^[1], adapted from the python implementation publicly available in Bullet. 
- 'EditEnvironments' contains tools to configure and change the hierarchies of joints and rigid bodies that form the training environments, including scaling them. To explore how to do this we recommend reading the documentaiton found in We recommend reading the Readme of the [Readme](<./EditEnvironments/README.md>) of the assembly.

[1] Tan, Liu & Turk (2011) Stable Proportional-Derivative Controllers
[DOI](https://dl.acm.org/doi/10.1109/MCG.2011.30)
