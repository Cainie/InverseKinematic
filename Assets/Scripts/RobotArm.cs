using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotArm : MonoBehaviour {

	[SerializeField] private RobotJoint[] _joints;
	public float SamplingDistance = 5;
	public float LearningRate = 100;
	public float DistanceThreshold = 0.2f;
	[SerializeField] private GameObject _target;
	private float[] _angles;

	private void Start()
	{
		_angles = new float[_joints.Length];
		
		for (var i = 0; i < _joints.Length; i++)
		{
			var joint = _joints[i];
			
			if (Math.Abs(joint.Axis.x - 1) < 0.001){
				_angles[i] = joint.transform.localRotation.x;
			}
			else if (Math.Abs(joint.Axis.y - 1) < 0.001){
				_angles[i] = joint.transform.localRotation.y;

			}
			else if (Math.Abs(joint.Axis.z - 1) < 0.001){
				_angles[i] = joint.transform.localRotation.z;

			}
		}
	}

	private void FixedUpdate()
	{
		InverseKinematics(_target.transform.position,_angles);
		for (var i = 0; i < _joints.Length; i++)
		{
			var joint = _joints[i];
			if (Math.Abs(joint.Axis.x - 1) < 0.001)
			{
				Debug.Log("x");
				joint.transform.localEulerAngles = new Vector3(_angles[i],0,0);
			}
			else if (Math.Abs(joint.Axis.y - 1) < 0.001){
				Debug.Log("y");
				joint.transform.localEulerAngles = new Vector3(0,_angles[i],0);
			}
			else if (Math.Abs(joint.Axis.z - 1) < 0.001){
				Debug.Log("z");
				joint.transform.localEulerAngles = new Vector3(0,0,_angles[i]);
			}
		}
	}

	private Vector3 ForwardKinematic(float[] angles)
	{
		var prevPoint = _joints[0].transform.position;
		var rotation = Quaternion.identity;
		for (var i = 1; i < _joints.Length; i++)
		{
			rotation *= Quaternion.AngleAxis(angles[i - 1], _joints[i - 1].Axis);
			var nextPoint = prevPoint + rotation * _joints[i].StartOffset;
			prevPoint = nextPoint;
		}
		return prevPoint;
	}

	private float DistanceFromTarget(Vector3 target, float [] angles)
	{
		var point = ForwardKinematic(angles);
		return Vector3.Distance(point, target);
	}

	private float PartialGradient (Vector3 target, float[] angles, int i)
	{
		var angle = angles[i];
		var fFromX = DistanceFromTarget(target, angles);
		angles[i] += SamplingDistance;
		var fFromXPlusD = DistanceFromTarget(target, angles);
		var gradient = (fFromXPlusD - fFromX) / SamplingDistance;
		angles[i] = angle;
		return gradient;
	}
	
	
	public void InverseKinematics (Vector3 target, float [] angles)
	{
		if (DistanceFromTarget(target, angles) < DistanceThreshold)
			return;
		for (var i = _joints.Length -1; i >= 0; i --)
		{
			var gradient = PartialGradient(target, angles, i);
			angles[i] -= LearningRate * gradient;
			if (DistanceFromTarget(target, angles) < DistanceThreshold)
				return;
		}
	}
}
