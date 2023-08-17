using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MapRagdoll2Anim : MonoBehaviour

//this class does exactly the symetrical of MocapControllerArtanim: it maps animations from a ragdoll to a rigged character
{

	[SerializeField]
	ArticulationBody _articulationBodyRoot;

	List<Transform> _animTransforms;
	List<Transform> _ragdollTransforms;

	//to generate an environment automatically from a rigged character and an animation (see folder ROM-extraction)
	public ArticulationBody ArticulationBodyRoot
	{
		set => _articulationBodyRoot = value;
		get => _articulationBodyRoot;
	}

	// use LateUpdate as physics runs at 2x and we only need to run once per render
	private void LateUpdate()
	{
		// MimicAnimationArtanim();
		if (_animTransforms == null)
		{
			var ragdollTransforms = 
				_articulationBodyRoot.GetComponentsInChildren<Transform>()
				.Where(x=>x.name.StartsWith("articulation"))
				.ToList();
			var ragdollNames = ragdollTransforms
				.Select(x=>x.name)
				.ToList();
			var animNames = ragdollNames
				.Select(x=>x.Replace("articulation:",""))
				.ToList();
			var animTransforms = animNames
				.Select(x=>GetComponentsInChildren<Transform>().FirstOrDefault(y=>y.name == x))
				.Where(x=>x!=null)
				.ToList();
			_animTransforms = new List<Transform>();
			_ragdollTransforms = new List<Transform>();
			// first time, copy position and rotation
			foreach (var animTransform in animTransforms)
			{
				var ragdollTransform = ragdollTransforms
					.First(x=>x.name == $"articulation:{animTransform.name}");
				animTransform.position = ragdollTransform.position;
				animTransform.rotation = ragdollTransform.rotation;
				_animTransforms.Add(animTransform);
				_ragdollTransforms.Add(ragdollTransform);
			}
		}
		// copy position for root (assume first target is root)
		_animTransforms[0].position = _ragdollTransforms[0].position;
		// copy rotation
		for (int i = 0; i < _animTransforms.Count; i++)
		{
			_animTransforms[i].rotation = _ragdollTransforms[i].rotation;
		}	
	}
}
