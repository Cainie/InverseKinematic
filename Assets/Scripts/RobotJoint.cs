using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;

public class RobotJoint : MonoBehaviour
{

	
	[FormerlySerializedAs("_axis")] public Vector3 Axis;
	[FormerlySerializedAs("_startOffset")] public Vector3 StartOffset;
	
	
	private void Awake()
	{
		StartOffset = transform.localPosition;
	}

}


