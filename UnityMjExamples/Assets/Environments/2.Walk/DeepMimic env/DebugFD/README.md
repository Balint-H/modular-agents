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

WHERE DOES THIS PROBLEM COME FROM?



To be able to compare the observations, I also create a build  from scene *5.StraightWalk_DeepMimic_BOTHobs.unity* and launch a training with instruction: 

```powershell
(marathon-envs) artanim@artanim-train-01:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml --run-id=run4025-WalkFD5-t1 --env=envs/WalkFD5/WalkFD --time-scale=1 
```

To make extra sure the problem doesn't come from the FD rewards, I create another scene       ( *6.StraightWalk_DeepMimic_FDobs_pupetRewards.unity* ) where I use the FD observatoins, but the FD rewards. Specifically:

```powershell
(marathon-envs) artanim@artanim-train-02:~/marathon-training$ mlagents-learn config/trainWalk2Target.yaml  --run-id=run4026-WalkFD6-t2   --time-scale=1 --env=envs/WalkFD6/WalkFD 
```
