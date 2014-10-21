using UnityEngine;
using System.Collections;

public class LocomotionCtrlScript : MonoBehaviour {
	public Animator animator = null;
	public float speed;
	public Vector3 vel = new Vector3(0, 0, 0);
	public Vector3 target = new Vector3(0, 0, 0);
	public bool refresh = false;
	public NavMeshAgent nma = null;
	public bool inPosition = false;
	// Use this for initialization
	void Start () {
		animator = GetComponent<Animator>();
		speed = 2.0f;
		refresh = true;
		nma = GetComponent<NavMeshAgent>();
		if (!target.Equals(transform.position)) {
			//nma.SetDestination(target);	
			//nma.stoppingDistance = 0.1f;
		}
	}
	
	// Update is called once per frame
	void Update () {
		if (animator) {
			float distance = Vector3.Distance(transform.position, target);
			if(distance < 0.1) {
				//nma.enabled = false;
				//nma.speed = 0.0f;
				vel.x = 0;
				vel.y = 0;
				vel.z = 0;
				inPosition = true;
			}
			speed = Vector3.Magnitude(vel);
			animator.SetFloat("Speed", speed);
		}
	}
}
