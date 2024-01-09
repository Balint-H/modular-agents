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
