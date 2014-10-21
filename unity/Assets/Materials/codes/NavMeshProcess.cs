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

	[DllImport("HybridFrameworkDLL", CallingConvention = CallingConvention.Cdecl)]
	private static extern void SetAgentCorridor(int aid, int cornerNum, float[] corners);
	
	//
	private List<GameObject> agents;
	private Texture[] textures;
	private Color[] colors;
	private int counter;
	// Use this for initialization
	void Start () {
		agents = new List<GameObject>();
		counter = 0;
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

		bool res = Init(1000, 0.3f, 20.0f, vertNum, verts, indNum, inds);
		SetDensityThreshold(1.0f);

	}
	void OnGUI () {
		if (GUI.Button(new Rect(10, 10, 200, 40), "Select Agents File")) {
			counter = 0;
			int aid = 0;
			string fileName = "corners.txt";
			StreamWriter sw = File.CreateText(fileName);

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
						Vector3 position = new Vector3(pos[0], pos[1], pos[2]);
						Vector3 destination = new Vector3(target[0], target[1], target[2]);
						// compute the rotation matrix and convert it to queternion
						Vector3 zaxis = new Vector3(0, 0, 1);
						Vector3 dir = (destination - position);
						dir.Normalize();
						float theta = Vector3.Angle(zaxis, dir);
						//theta = theta * 180 / (float)Math.PI;
						Quaternion q = Quaternion.AngleAxis(theta, Vector3.up);

						GameObject agent = Instantiate(Resources.Load("prefab/HMAgent", typeof(GameObject)), position, q) as GameObject;

						agent.transform.position = position;
						agent.GetComponent<LocomotionCtrlScript>().target = destination;
						//agent.GetComponent<NavMeshAgent>().SetDestination(destination);
						NavMeshPath path = new NavMeshPath();
						NavMesh.CalculatePath(position, destination, -1, path);
						int cornerNum = path.corners.Length;
						float[] corners = new float[cornerNum*3];
						for(int i=0; i<cornerNum; i++)
						{
							corners[i*3] = path.corners[i].x;
							corners[i*3+1] = path.corners[i].y;
							corners[i*3+2] = path.corners[i].z;
						}

						agent.GetComponentInChildren<Renderer>().material.mainTexture = textures[color];
						agent.GetComponentInChildren<Renderer>().material.color = colors[color];
						sw.WriteLine(cornerNum);
						for(int i=0; i<cornerNum; i++) {
							sw.WriteLine(corners[i*3]);
							sw.WriteLine(corners[i*3+1]);
							sw.WriteLine(corners[i*3+2]);
						}
						sw.Flush();
						float maxNeighborDist = (float)Convert.ToDouble(parse[7]);
						int maxNeighborNum = Convert.ToInt32(parse[8]);
						float planHorizon = (float)Convert.ToDouble(parse[9]);
						float radius = (float)Convert.ToDouble(parse[10]);	// thi is the comfort radius
						float maxSpeed = (float)Convert.ToDouble(parse[11]);

						AddAgent(pos, maxNeighborDist, maxNeighborNum, planHorizon, radius, maxSpeed, target, vel, color);
						SetAgentCorridor(aid, cornerNum, corners);
						//agent.GetComponent<NavMeshAgent>().enabled = false;
						agents.Add(agent);
						aid++;
					}
				}
				sw.Close();
			}
		}
	}

	// Update is called once per frame
	void Update () {
		Vector3 zaxis = new Vector3(0, 0, 1);
		int agentNum = agents.Count;
		if(agentNum > 0) {
			if(counter > 600) {
				//return;
			}
			counter++;
			int[] agentIds = new int[agentNum];
			float[] pos = new float[3*agentNum];
			float[] vel = new float[3*agentNum];
			for(int i=0; i<agentNum; i++) {
				agentIds[i] = i;
				//agents[i].GetComponent<NavMeshAgent>().enabled = false;
				Vector3 position = agents[i].transform.position;
				Vector3 velocity = agents[i].GetComponent<LocomotionCtrlScript>().vel;
				pos[i*3+0] = position.x;	pos[i*3+1] = position.y;	pos[i*3+2] = position.z;
				vel[i*3+0] = velocity.x;	vel[i*3+1] = velocity.y;	vel[i*3+2] = velocity.z;
			}
			float dt = Time.deltaTime;
			string fileName = "update.txt";
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
			}
			Update(dt, agentNum, agentIds, pos, vel);
			for(int i=0; i<agentNum; i++) {
				if(agents[i].GetComponent<LocomotionCtrlScript>().inPosition == true) {
					continue;
				}

				agents[i].transform.position = new Vector3(pos[i*3+0], pos[i*3+1], pos[i*3+2]);
				Vector3 velocity = new Vector3(vel[i*3+0], vel[i*3+1], vel[i*3+2]);
				agents[i].GetComponent<LocomotionCtrlScript>().vel = velocity;
				Vector3 dest = agents[i].GetComponent<LocomotionCtrlScript>().target;
				velocity.Normalize();
				dest.Normalize();
				float theta = Vector3.Angle(zaxis, velocity);
				//theta = theta * 180 / (float)Math.PI;
				//agents[i].transform.RotateAround(agents[i].transform.position, Vector3.up, theta);
				agents[i].transform.rotation = Quaternion.AngleAxis(-theta, Vector3.up);
				//agents[i].transform.forward = velocity;
				//agents[i].GetComponent<NavMeshAgent>().enabled = true;
			}
			//if(counter % 30 == 0) {	// reset path every 200 frames
				for(int i=0; i<agentNum; i++) {
					agents[i].transform.position = new Vector3(pos[i*3+0], pos[i*3+1], pos[i*3+2]);
					Vector3 target = agents[i].GetComponent<NavMeshAgent>().destination;
					//agents[i].GetComponent<NavMeshAgent>().enabled = true;
					NavMeshPath path = new NavMeshPath();
					NavMesh.CalculatePath(agents[i].transform.position, target, -1, path);
					int cornerNum = path.corners.Length;
					float[] corners = new float[cornerNum*3];
					for(int j=0; j<cornerNum; j++)
					{
						corners[j*3] = path.corners[i].x;
						corners[j*3+1] = path.corners[i].x;
						corners[j*3+2] = path.corners[i].x;
					}
					SetAgentCorridor(i, cornerNum, corners);
				}
			//}
		}

	}
}
