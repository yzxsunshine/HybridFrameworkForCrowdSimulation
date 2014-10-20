using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;

namespace HFDLLTest
{
    class Program
    {
        [DllImport("HybridFrameworkDLL", CallingConvention = CallingConvention.Cdecl)]
        private static extern bool Init(int maxAgents, float renderAgentRadius, float gridSize, int vn, float[] verts, int fn, int[] inds);

        [DllImport("HybridFrameworkDLL", CallingConvention = CallingConvention.Cdecl)]
        private static extern int AddAgent(float[] pos
		    , float maxNeighborDist, int maxNeighborNum, float planHorizon, float radius, float maxSpeed	// simulation parameters
            , float[] target, float[] vel, int color);

	    /// Updates the specified agent's configuration.
	    ///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	    ///  @param[in]		params	The new agent configuration.
        [DllImport("HybridFrameworkDLL", CallingConvention = CallingConvention.Cdecl)]
        private static extern void UpdateAgentParameters(int idx, float maxNeighborDist, int maxNeighborNum, float planHorizon, float radius, float maxSpeed);

	    /// Removes the agent from the crowd.
	    ///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
        [DllImport("HybridFrameworkDLL", CallingConvention = CallingConvention.Cdecl)]
	    private static extern void RemoveAgent(int idx);

	    /// Updates the steering and positions of all agents.
	    ///  @param[in]		dt		The time, in seconds, to update the simulation. [Limit: > 0]
	    ///  @param[out]	debug	A debug object to load with debug information. [Opt]
        [DllImport("HybridFrameworkDLL", CallingConvention = CallingConvention.Cdecl)]
        private static extern void Update(float dt, int agentNum, int[] agentIds, float[] positions, float[] velocities);

        [DllImport("HybridFrameworkDLL", CallingConvention = CallingConvention.Cdecl)]
	    private static extern void Clear();

        static void Main(string[] args)
        {
            int vn = 4;
            float[] verts = new float[vn*3];
            verts[0] = -4.0f;   verts[1] = 0.0f;    verts[2] = -4.0f;
            verts[3] = 4.0f;    verts[4] = 0.0f;    verts[5] = -4.0f;
            verts[6] = 4.0f;    verts[7] = 0.0f;    verts[8] = 4.0f;
            verts[9] = -4.0f;   verts[10] = 0.0f;   verts[11] = 4.0f;
            int fn = 2;
            int[] inds = new int[fn*3];
            inds[0] = 0;    inds[1] = 1;    inds[2] = 3;
            inds[3] = 3;    inds[4] = 1;    inds[5] = 2;
            float renderRadius = 0.5f;
            int maxAgentNum = 100;
            float gridSize = 4.0f;
            Init(maxAgentNum, renderRadius, gridSize, vn, verts, fn, inds);

            float[] pos = new float[3];
            pos[0] = 0.0f; pos[1] = 0.0f; pos[2] = 0.0f;
            float[] target = new float[3];
            target[0] = 1.0f; target[1] = 1.0f; target[2] = 1.0f;
            float[] vel = new float[3];
            vel[0] = 0.1f; vel[1] = 0.1f; vel[2] = 0.1f;
            AddAgent(pos, 2.0f, 10, 5.0f, 0.2f, 1.0f, target, vel, 0);

            int[] ids = new int[1];
            ids[0] = 0;
            Update(0.033f, 1, ids, pos, vel);
        }
    }
}
