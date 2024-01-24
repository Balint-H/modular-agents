The following scenes are modifications of the scene *StraightWalk_DeepMimic.unity*, trying to figure out why the training with Finite Differences (FD) does not work when we remove the pupet. The scenes are the following:

- *1.StraightWalk_DeepMimic_FD.unity* is the scene that we want to work. It uses:
  
  - FD observations and rewards
  
  - New reset system
  
  - No pupet
  
  - **it does not learn** , we recheck with:
    
    ```shell
    (marathon-envs) artanim@artanim-train-01:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4021-WalkFD-t1 --env=envs/WalkFD1/WalkFD --time-scale=1    
    ```

- *2.StraightWalk_DeepMimic_FDinpupet__debug4train_newreset_.unity* combines:
  
  - FD observations and rewards 
    (it shows both rewards using pupet and without using it)
  
  - New reset system
  
  - A pupet with the constraints (that only act  at reset?)
  
  - **it does learn**

```shell
(marathon-envs) artanim@artanim-train-02:~/marathon-training$  mlagents-learn config/trainWalk2Target.yaml --run-id=run4022-WalkFD-t1 --env=envs/WalkFD2/WalkFD --time-scale=1
```

- *3.StraightWalk_DeepMimic_FDnopupet__debug4train_oldreset.unity* combines:
  
  - FD observations and rewards
  
  - Old reset system
  
  - No pupet
  
  - **not sure if it learns** 

```shell
(marathon-envs) artanim@artanim-train-03:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4023-WalkFD-t1 --env=envs/WalkFD3/WalkFD --time-scale=1
```

For comparison, we also compile the reference scene *StraightWalk_DeepMimic.unity*, and train it with:

```shell
(marathon-envs) artanim@artanim-train-04:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4020-WalkFD-t1 --env=envs/WalkFD0/WalkFD --time-scale=1
```

08.01.2024

The differences are spectacular

I try again with a simple modification of the reference implementation, just to log the observations and the rewards.

4.StraightWalk_DeepMimic_onlyFDobs.unity

I generate the observations. At simple inspection they look quite similar, but not the same (i.e., the observations from FD and from the Pupet), which seems to make sense. I need to show them in a python script, to compare trajectories.

Also, I notice the model that worked on the reference implementation does not work here,when I add the infrastructure to record observations and rewards on top of the environment that already works. To make sure I am not messing up something I relaunch a training, to check if I get the same result.

09.01.2024

That training seems to introduce a SIGNIFICANT difference in the training outcome.

To make sure I launched a training with only adding the obs (and not using them), I relaunch the following training:

Build created from *4.StraightWalk_DeepMimic_onlyFDobs.unity* in commit

Commit: 9533da323f901bd0a62b181f13b04128e3cbc401 [9533da3]
Parents: 07fa6f5e70
Author: Joan Llobera <joan.llobera@artanim.ch>
Date: Tuesday, 9 January 2024 14:36:00
Committer: Joan Llobera

with instruction: 

```powershell
(marathon-envs) artanim@artanim-train-04:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4024-WalkFD4v2-t3 --env=envs/WalkFD4v2/WalkFD --time-scale=1 
```

The partial results suggest there is a problem in getting the right rewards. The orange line is the training reward for DeepMimic walk straight, and the pink one is for DeepMimic, just adding the extra FD observatoins in the calculation loop but not using them.

![2024-01-10-11-36-28-image.png](.\img\2024-01-10-11-36-28-image.png)

WHERE DOES THIS PROBLEM COME FROM? unknown

To be able to compare the observations, I also create a build  from scene *5.StraightWalk_DeepMimic_BOTHobs.unity* and launch a training with instruction: 

```powershell
(marathon-envs) artanim@artanim-train-01:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4025-WalkFD5-t1 --env=envs/WalkFD5/WalkFD --time-scale=1 
```

To make extra sure the problem doesn't come from the FD rewards, I create another scene       ( *6.StraightWalk_DeepMimic_FDobs_pupetRewards.unity* ) where I use the FD observatoins, but the Pupet rewards. Specifically:

```powershell
(marathon-envs) artanim@artanim-train-02:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml  --run-id=run4026-WalkFD6-t2   --time-scale=1 --env=envs/WalkFD6/WalkFD 
```

![2024-01-10-12-00-01-image.png](.\img\2024-01-10-12-00-01-image.png)

Above we have:

-In green the training of Deep Mimic with the pupet

-In grey, the training of Deep Mimic with the pupet, but using observations from FD calculations

-In cyan training of Deep Mimic with the pupet, using observations BOTH from FD and the pupet (see the marginal increment)

12.01.2024

trying to understand why the rewards that we get from the finite differences do not work, I launch the following tests:

WalkFD7:

invert the localRotation of the ball joints

Launch instruction:

```
(marathon-envs) artanim@artanim-train-01:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4027-WalkFD7-t1 --env=envs/WalkFD7/WalkFD --time-scale=1 
```

WalkFD8: 

invert the localRotatoin of the ball joints and use the new decomposition for sibling hinge joints  (check this: it is actually re-inverted in the reward, the operation was undone in the reward )

```shell
(marathon-envs) artanim@artanim-train-02:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml  --run-id=run4026-WalkFD8-t1   --time-scale=1 --env=envs/WalkFD8/WalkFD 
```

WalkFD9:

do not invert localRotation in ball joints and use hte new decomposition for sibling hinge joints (check this: localRotation is actually inverted in the reward, the operation was redone in the reward )

```shell
(marathon-envs) artanim@artanim-train-03:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4029-WalkFD9-t1 --env=envs/WalkFD9/WalkFD --time-scale=1
```

WalkFD10:

do not invert localRotation in ball joints, use the decomposition for sibling  hinge joints and do not invert localRotation in the reward

```
(marathon-envs) artanim@artanim-train-04:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4030-WalkFD10-t1 --env=envs/WalkFD10/WalkFD --time-scale=1
```

16.01.2024

WalkFD11:

fix to the way the sibling hinge joints have their rotations calculated

(marathon-envs) artanim@artanim-train-01:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4031-WalkFD11-t1 --env=envs/WalkFD11/WalkFD --time-scale=1 

19.01.2024

Training set ups using FDReward:

1. No pupet, new reset

scene: **1.StraightWalk_DeepMimic_FD**

build: **WalkFD41**

training instruction:

```shell
(marathon-envs) artanim@artanim-train-01:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4041-WalkFD41-t1 --env=envs/WalkFD41/WalkFD --time-scale=1
```

2. Pupet, new reset
   
   scene: **2.StraightWalk_DeepMimic_FDinpupet_debug4train_newreset**
   
   build: **WalkFD42**
   
   training instruction:
   
   ```shell
   (marathon-envs) artanim@artanim-train-02:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml  --run-id=run4042-WalkFD42-t1   --time-scale=1 --env=envs/WalkFD42/WalkFD 
   ```
   
   note: this environment uses wrists instead of hands for the End Effector reward component

3. No pupet, old reset

scene: **3.StraightWalk_DeepMimic_FDnopupet_debug4train_oldreset**

build: **WalkFD43**

training instruction:

```shell
(marathon-envs) artanim@artanim-train-03:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4043-WalkFD43-t1 --env=envs/WalkFD43/WalkFD --time-scale=1
```

4. Pupet, pupet rewards, new reset
   
   scene: **2.2.StraightWalk_DeepMimic_Rewardsinpupet_newreset.unity**
   
   build: **WalkFD44**
   
   training instruction:
   
   ```shell
   (marathon-envs) artanim@artanim-train-04:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4044-WalkFD44-t1 --env=envs/WalkFD44/WalkFD --time-scale=1 
   ```
   
   This scene is to double check that using the old method with the new reset method still works

The 4 environments have been launched with the versoin of the project in commit *Commit: fadba8f867a23e18b26ddf7d04f429b1b0363bc6 [fadba8f]*

23.01.2024

The rewards for cases 42 and 44 do not work very well, but are very similar and still better than 41 and 43.

I try with pupet and old reset, but with FD rewards.

To launch with pupet and old reset, only changing the FD rewards

scene: **9.StraightWalk_deepMimic_FDRewards**

build: **WalkFD45**

*Commit: 96b3d91549208206347dd31cf26db971cc189d9f [96b3d91]* 

training instruction:

```shell
(marathon-envs) artanim@artanim-train-01:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4045-WalkFD45-t2 --env=envs/WalkFD45/WalkFD --time-scale=1 
```

Exactly the same environment switching the reward calculation:

scene: **9.2.StraightWalk_deepMimic_PupetRewards**

build: **WalkFD46**

*Commit:  f12377454745c372f17716bd0cdb5c4133749de5 [f123774]* 

training instruction:

```shell
(marathon-envs) artanim@artanim-train-02:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4046-WalkFD46-t1 --env=envs/WalkFD46/WalkFD --time-scale=1 
```

Set up the new reset and the removal of the ragdoll, but using EXACTLY the same limbs ofr the rewards, I get:

scene: **9.3.StraightWalk_deepMimic_NoPupet**

build: **WalkFD47**

training:

```
mlagents-learn config/trainWalk2Target.yaml --run-id=run4047-WalkFD47-t1 --env=envs/WalkFD47/WalkFD --time-scale=1
```

After several trials,  I change one thing in the way the FD Ball joints work, and relaunch a training using:

scene: **9.3.StraightWalk_deepMimic_NoPupet**

build: **WalkFD50**

training: 

```
(marathon-envs) artanim@artanim-train-03:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4050-WalkFD50-t2 --env=envs/WalkFD50/WalkFD --time-scale=1
```

extra training removing the *ForwardKinematics();* in line 77 of MjFiniteDifferenceManager:

```shell
(marathon-envs) artanim@artanim-train-04:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4051-WalkFD51-t1 --env=envs/WalkFD51/WalkFD --time-scale=1
```

24.01.2024

relaunched the previous (without forwardKinematics) with 800 steps to have a comparable metric. Insturction:

```shell
(marathon-envs) artanim@artanim-train-03:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4052-WalkFD52-t1 --env=envs/WalkFD52/WalkFD --time-scale=1
```

Double check about the root joint (I now add it in observations):

```shell
(marathon-envs) artanim@artanim-train-04:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4053-WalkFD53-t1 --env=envs/WalkFD53/WalkFD --time-scale=1
```
