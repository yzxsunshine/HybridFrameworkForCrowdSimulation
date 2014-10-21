using UnityEngine;
using System.Collections;

public class LocomotionCtrlScript : MonoBehaviour {
	public Animator animator = null;
	public float speed;
	public Vector3 target = new Vector3(0, 0, 0);
	public bool refresh = false;
	public NavMeshAgent nma = null;
	// Use this for initialization
	void Start () {
		animator = GetComponent<Animator>();
		speed = 2.0f;
		refresh = true;
		nma = GetComponent<NavMeshAgent>();
		if (!target.Equals(transform.position)) {
			nma.SetDestination(target);	
			nma.stoppingDistance = 0.1f;
		}
	}
	
	// Update is called once per frame
	void Update () {
		if (animator) {
			float distance = Vector3.Distance(transform.position, target);
			if(distance < 0.1) {
				nma.enabled = false;
				nma.speed = 0.0f;
			}
			speed = Vector3.Magnitude(nma.velocity);
			animator.SetFloat("Speed", speed);
		}
	}
}
