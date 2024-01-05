





The following scenes are modifications of the scene *StraightWalk_DeepMimic.unity*, trying to figure out why the training with Finite Differences (FD) does not work when we remove the pupet. The scenes are the following:

- *1.StraightWalk_DeepMimic_FD.unity* is the scene that we want to work. It uses:
  
  - FD observations and rewards
  
  - New reset system
  
  - No pupet
  
  - **it does not learn**

- *2.StraightWalk_DeepMimic_FDinpupet__debug4train_newreset_.unity* combines:
  
  - FD observations and rewards 
    (it shows both rewards using pupet and without using it)
  
  - New reset system
  
  - A pupet with the constraints (that only act  at reset?)
  
  - **it does learn**

- *3.StraightWalk_DeepMimic_FDnopupet__debug4train_oldreset.unity* combines:
  
  - FD observations and rewards
  
  - Old reset system
  
  - No pupet
  
  - **not sure if it learns** 




