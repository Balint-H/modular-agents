# Training

Control models of **Modular Agents** environments can be trained through any of the methods described in the [ML-Agents toolkit documentation](https://unity-technologies.github.io/ml-agents/Training-ML-Agents/). We recommend using a [built executable environment](https://unity-technologies.github.io/ml-agents/Learning-Environment-Executable/) for training. 
We use the following folder structure:

- **envs**: Build the environment executable here (either Windows or Linux work, depending on your platform).
- **config**: This folder contains the configuration files that work for a build using the example projects. 
- **results**: The results of the training (the control policy model and the learning metrics) will appear in this folder.

To install the learning environment, you can use file *environment.yaml* in conda to install the ml-agents package. The installation procedure is analogous to the one in the older *Marathon* project. The instructions can be found [`here`](https://github.com/joanllobera/marathon-envs/blob/feautre/cleanup-mujoco/docs/installation.md). 

To train a simple environment you can make a build from the DeepMimic walk scene and then do:

```
(ml-agents) PS D:\PUT_HERE_PATH_TO_REPO\Training>  mlagents-learn config/trainMujoco.yaml --run-id=DeepMimic_Walk --env=envs/UnityMjExamples --num-envs=8
```


A summary of how to train an environment is available [`here`](https://github.com/joanllobera/marathon-envs/blob/feautre/cleanup-mujoco/docs/marathon-controller-training.md) 



## How to train on  cloud infrastructure

The instructions below have worked in google cloud SDK shell, after installing it and introducing the zone (*europe-west1-d*)

### 1. Upload environment

You can use the `scp` command to copy a zip file to the server. For example:

```
D:\PUT_HERE_PATH_TO_REPO\envs>gcloud compute scp Throw36PhysXMujocoL.zip instance-2:.
```

### 2. Connect through SSH:

To connect use:

```
gcloud compute ssh --zone=europe-west1-d  instance-2
```

Notice that the zone may not be needed if you use the one set up by default.

Details:

[Connect to Linux VMs using Google tools &nbsp;|&nbsp; Compute Engine Documentation &nbsp;|&nbsp; Google Cloud](https://cloud.google.com/compute/docs/instances/connecting-to-instance#gcloud) 

Instructions for setting up Anaconda on the cloud:

https://medium.com/google-cloud/set-up-anaconda-under-google-cloud-vm-on-windows-f71fc1064bd7 

```
> sudo apt get && sudo apt upgrade
> sudo apt install wget
```

Find latest linux install here: https://repo.anaconda.com/archive/

for example, to target: **[Anaconda3-2022.10-Linux-x86_64.sh](https://repo.anaconda.com/archive/Anaconda3-2022.10-Linux-x86_64.sh)**

```
> wget https://repo.anaconda.com/archive/Anaconda3-2022.10-Linux-x86_64.sh
```

To connect and do things as root simply use the previous method, and to connect as root

[Connecting to Linux VMs using advanced methods &nbsp;|&nbsp; Compute Engine Documentation &nbsp;|&nbsp; Google Cloud](https://cloud.google.com/compute/docs/instances/connecting-advanced#root) 
