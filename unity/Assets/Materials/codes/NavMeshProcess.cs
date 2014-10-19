using UnityEngine;
using System.Collections;

// this code is used to convert scene(the plane it was attached on) and the navigation mesh into readable data for our hybrid framework dll
public class NavMeshProcess : MonoBehaviour {

	// Use this for initialization
	void Start () {
		// read scene data
		Vector3 boxMin = renderer.bounds.min;
		Vector3 boxMax = renderer.bounds.max;
		// read navigation mesh
		NavMeshTriangulation navMeshTris = NavMesh.CalculateTriangulation();
		int indNum = navMeshTris.indices.Length;
		int triNum = navMeshTris.vertices.Length;
	}
	
	// Update is called once per frame
	void Update () {
	
	}
}
