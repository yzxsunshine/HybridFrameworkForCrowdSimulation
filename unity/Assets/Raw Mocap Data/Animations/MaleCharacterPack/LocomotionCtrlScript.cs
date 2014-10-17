using UnityEngine;
using System.Collections;

public class LocomotionCtrlScript : MonoBehaviour {
	Animator animator;
	float speed;
	public Transform targetObj = null;
	// Use this for initialization
	void Start () {
		animator = GetComponent<Animator>();
		speed = 2.0f;
		if (targetObj != null) {
			GetComponent<NavMeshAgent>().destination = targetObj.position;	
			GetComponent<NavMeshAgent>().stoppingDistance = 0.01f;
		}
	}
	
	// Update is called once per frame
	void Update () {
		if (animator) {
			float distance = Vector3.Distance(transform.position, targetObj.position);
			if(distance < 0.5) {
				GetComponent<NavMeshAgent>().enabled = false;
				GetComponent<NavMeshAgent>().speed = 0.0f;
			}
			speed = GetComponent<NavMeshAgent>().speed;
			animator.SetFloat("Speed", speed);
		}
	}
}
