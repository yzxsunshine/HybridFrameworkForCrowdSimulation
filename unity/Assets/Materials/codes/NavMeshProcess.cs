using UnityEngine;
using System.Collections;
using System;
using System.Runtime.InteropServices;
using System.IO;
using UnityEditor;
using System.Collections.Generic;

// this code is used to convert scene(the plane it was attached on) and the navigation mesh into readable data for our hybrid framework dll
public class NavMeshProcess : MonoBehaviour {
	[DllImport("HybridFrameworkDLL", CallingConvention = CallingConvention.Cdecl)]
	private static extern bool Init(int maxAgents, float renderAgentRadius, float gridSize, int vn, float[] verts, int fn, int[] inds);
	
	[DllImport("HybridFrameworkDLL", CallingConvention = CallingConvention.Cdecl)]
	private static extern int AddAgent(float[] pos
	                                   , float maxNeighborDist, int maxNeighborNum, float planHorizon, float radius, float maxSpeed	// simulation parameters
	                                   , float[] target, float[] vel, int color);

	[DllImport("HybridFrameworkDLL", CallingConvention = CallingConvention.Cdecl)]
	private static extern void UpdateAgentParameters(int idx, float maxNeighborDist, int maxNeighborNum, float planHorizon, float radius, float maxSpeed);

	[DllImport("HybridFrameworkDLL", CallingConvention = CallingConvention.Cdecl)]
	private static extern void RemoveAgent(int idx);

	[DllImport("HybridFrameworkDLL", CallingConvention = CallingConvention.Cdecl)]
	private static extern void Update(float dt, int agentNum, int[] agentIds, float[] positions, float[] velocities);
	
	[DllImport("HybridFrameworkDLL", CallingConvention = CallingConvention.Cdecl)]
	private static extern void Clear();
	
	[DllImport("HybridFrameworkDLL", CallingConvention = CallingConvention.Cdecl)]
	private static extern void SetDensityThreshold(float thresh);
	
	//
	private List<GameObject> agents;
	private Texture[] textures;
	private Color[] colors;

	// Use this for initialization
	void Start () {
		agents = new List<GameObject>();
		// read scene data
		Vector3 boxMin = renderer.bounds.min;
		Vector3 boxMax = renderer.bounds.max;
		// read navigation mesh
		NavMeshTriangulation navMeshTris = NavMesh.CalculateTriangulation();

		int vertNum = navMeshTris.vertices.Length;
		float[] verts = new float[3 * vertNum];
		for(int i=0; i<vertNum; i++) {
			verts[i*3] = navMeshTris.vertices[i].x;
			verts[i*3+1] = navMeshTris.vertices[i].y;
			verts[i*3+2] = navMeshTris.vertices[i].z;
		}
		int[] inds = navMeshTris.indices;
		int indNum = inds.Length / 3;
		string fileName = "debug.txt";
		if(!File.Exists(fileName)) {
			StreamWriter sw = File.CreateText(fileName);
			sw.WriteLine(vertNum);
			for(int i=0; i<vertNum; i++) {
				sw.WriteLine(verts[i*3]);
				sw.WriteLine(verts[i*3+1]);
				sw.WriteLine(verts[i*3+2]);
			}
			sw.WriteLine(indNum);
			for(int i=0; i<indNum; i++) {
				sw.WriteLine(inds[i*3]);
				sw.WriteLine(inds[i*3+1]);
				sw.WriteLine(inds[i*3+2]);
			}
			sw.Close();
		}

		textures = new Texture[4];
		textures[0] = Resources.Load("textures/Vincent_green", typeof(Texture)) as Texture;
		textures[1] = Resources.Load("textures/Vincent_red", typeof(Texture)) as Texture;
		textures[2] = Resources.Load("textures/Vincent_gold", typeof(Texture)) as Texture;
		textures[3] = Resources.Load("textures/Vincent_blue", typeof(Texture)) as Texture;

		colors = new Color[4];
		colors[0] = Color.green;
		colors[1] = Color.red;
		colors[2] = Color.yellow;
		colors[3] = Color.blue;

		bool res = Init(1000, 1.0f, 20.0f, vertNum, verts, indNum, inds);
		SetDensityThreshold(0.1f);
		if(res) {
			GameObject agentsParent = GameObject.Find("Agents");
			//Type type = GameObject.Find("vincent0").GetType();
			/*for(int i=0; i<agents.Length; i++) {
				Vector3 position = agents[i].transform.position;
				Vector3 velocity = agents[i].velocity;
				float maxSpeed = agents[i].speed;
				float maxNeighborDist = 20.0f;
				int maxNeighborNum = 70;
				float planHorizon = 10.0f;
				float radius = 0.2f;	// thi is the comfort radius
				Vector3 destination = new Vector3(0, 0, 0);//agents[i].destination;
				int type = 0;
				float[] pos = new float[3];
				float[] vel = new float[3];
				float[] target = new float[3];
				pos[0] = position.x;	pos[1] = position.y;	pos[2] = position.z;
				vel[0] = velocity.x;	vel[1] = velocity.y;	vel[2] = velocity.z;
				target[0] = destination.x;	target[1] = destination.y;	target[2] = destination.z;
				AddAgent(pos, maxNeighborDist, maxNeighborNum, planHorizon, radius, maxSpeed, target, vel, type);
			}*/
		}
	}
	void OnGUI () {
		if (GUI.Button(new Rect(10, 10, 200, 40), "Select Agents File")) {
			string filePath = EditorUtility.OpenFilePanel("Agents File",Application.streamingAssetsPath,"txt");
			if(filePath.Length != 0) {
				StreamReader sr = new StreamReader(filePath);
				string line;
				while((line = sr.ReadLine()) != null) {
					string[] parse = line.Split(new char[]{' '}, StringSplitOptions.RemoveEmptyEntries);
					if(parse.Length == 12) {
						float[] pos = new float[3];
						float[] vel = new float[3];
						float[] target = new float[3];
						pos[0] = (float)Convert.ToDouble(parse[0]);	pos[1] = (float)Convert.ToDouble(parse[1]);	pos[2] = (float)Convert.ToDouble(parse[2]);
						vel[0] = 0;	vel[1] = 0;	vel[2] = 0;
						target[0] = (float)Convert.ToDouble(parse[3]);	target[1] = (float)Convert.ToDouble(parse[4]);	target[2] = (float)Convert.ToDouble(parse[5]);
						int color = Convert.ToInt32(parse[6]);

						GameObject agent = Instantiate(Resources.Load("prefab/HMAgent", typeof(GameObject))) as GameObject;
						agent.transform.position = new Vector3(pos[0], pos[1], pos[2]);
						agent.GetComponent<NavMeshAgent>().SetDestination(new Vector3(target[0], target[1], target[2]));
						agent.GetComponentInChildren<Renderer>().material.mainTexture = textures[color];
						agent.GetComponentInChildren<Renderer>().material.color = colors[color];
						agents.Add(agent);

						
						float maxNeighborDist = (float)Convert.ToDouble(parse[7]);
						int maxNeighborNum = Convert.ToInt32(parse[8]);
						float planHorizon = (float)Convert.ToDouble(parse[9]);
						float radius = (float)Convert.ToDouble(parse[10]);	// thi is the comfort radius
						float maxSpeed = (float)Convert.ToDouble(parse[11]);

						AddAgent(pos, maxNeighborDist, maxNeighborNum, planHorizon, radius, maxSpeed, target, vel, color);
					}
				}
			}
		}
	}

	// Update is called once per frame
	void Update () {

		int agentNum = agents.Count;
		if(agentNum > 0) {
			int[] agentIds = new int[agentNum];
			float[] pos = new float[3*agentNum];
			float[] vel = new float[3*agentNum];
			for(int i=0; i<agentNum; i++) {
				agentIds[i] = i;
				//agents[i].GetComponent<NavMeshAgent>().enabled = false;
				Vector3 position = agents[i].transform.position;
				Vector3 velocity = agents[i].GetComponent<NavMeshAgent>().velocity;
				pos[i*3+0] = position.x;	pos[i*3+1] = position.y;	pos[i*3+2] = position.z;
				vel[i*3+0] = velocity.x;	vel[i*3+1] = velocity.y;	vel[i*3+2] = velocity.z;
			}
			float dt = Time.deltaTime;
			/*string fileName = "update.txt";
			if(!File.Exists(fileName)) {
				StreamWriter sw = File.CreateText(fileName);
				sw.WriteLine(dt);
				sw.WriteLine(agentNum);
				for(int i=0; i<agentNum; i++) {
					sw.WriteLine(agentIds[i]);
					sw.WriteLine(pos[i*3]);
					sw.WriteLine(pos[i*3+1]);
					sw.WriteLine(pos[i*3+2]);
					sw.WriteLine(vel[i*3]);
					sw.WriteLine(vel[i*3+1]);
					sw.WriteLine(vel[i*3+2]);
				}
				sw.Close();
			}*/
			Update(dt, agentNum, agentIds, pos, vel);
			for(int i=0; i<agentNum; i++) {
				agents[i].transform.position = new Vector3(pos[i*3+0], pos[i*3+1], pos[i*3+2]);
				//agents[i].GetComponent<NavMeshAgent>().velocity = new Vector3(vel[i*3+0], vel[i*3+1], vel[i*3+2]);
				//agents[i].GetComponent<NavMeshAgent>().enabled = true;
			}
		}

	}
}
