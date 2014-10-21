using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using System.IO;

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

        [DllImport("HybridFrameworkDLL", CallingConvention = CallingConvention.Cdecl)]
        private static extern void SetDensityThreshold(float thresh);

        static void Main(string[] args)
        {
            /*
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
            */
            StreamReader sr = new StreamReader(@"../debug.txt");
            string line;
            line = sr.ReadLine();   // it's not safe to directly read, but as we only need to test this one time.. so be it
            int vn = Convert.ToInt32(line);
            float[] verts = new float[vn * 3];
            for (int i = 0; i < vn * 3; i++)
            {
                line = sr.ReadLine();
                verts[i] = (float)Convert.ToDouble(line);
            }
            line = sr.ReadLine();
            int fn = Convert.ToInt32(line);
            int[] inds = new int[fn * 3];
            for (int i = 0; i < fn * 3; i++)
            {
                line = sr.ReadLine();
                inds[i] = Convert.ToInt32(line);
            }
            float renderRadius = 0.5f;
            int maxAgentNum = 1000;
            float gridSize = 20.0f;
            Init(maxAgentNum, renderRadius, gridSize, vn, verts, fn, inds);
           
           
            StreamReader sr2 = new StreamReader(@"../Tests/group_group.txt");
            while ((line = sr2.ReadLine()) != null)
            {
                string[] parse = line.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                if (parse.Length == 12)
                {
                    float[] pos = new float[3];
                    float[] vel = new float[3];
                    float[] target = new float[3];
                    pos[0] = (float)Convert.ToDouble(parse[0]); pos[1] = (float)Convert.ToDouble(parse[1]); pos[2] = (float)Convert.ToDouble(parse[2]);
                    vel[0] = 0; vel[1] = 0; vel[2] = 0;
                    target[0] = (float)Convert.ToDouble(parse[3]); target[1] = (float)Convert.ToDouble(parse[4]); target[2] = (float)Convert.ToDouble(parse[5]);

                    int color = Convert.ToInt32(parse[6]);

                    float maxNeighborDist = (float)Convert.ToDouble(parse[7]);
                    int maxNeighborNum = Convert.ToInt32(parse[8]);
                    float planHorizon = (float)Convert.ToDouble(parse[9]);
                    float radius = (float)Convert.ToDouble(parse[10]);	// thi is the comfort radius
                    float maxSpeed = (float)Convert.ToDouble(parse[11]);

                    AddAgent(pos, maxNeighborDist, maxNeighborNum, planHorizon, radius, maxSpeed, target, vel, color);
                }
            }
            int count = 0;
            StreamReader sr3 = new StreamReader(@"../update.txt");
            line = sr3.ReadLine();
            float dt = (float)Convert.ToDouble(line);
            line = sr3.ReadLine();
            int agentNum = Convert.ToInt32(line);
            int[] ids = new int[agentNum];
            float[] poss = new float[3 * agentNum];
            float[] vels = new float[3 * agentNum];
            for (int i = 0; i < agentNum; i++)
            {
                line = sr3.ReadLine();
                ids[i] = Convert.ToInt32(line);

                line = sr3.ReadLine();
                poss[i * 3] = (float)Convert.ToDouble(line);
                line = sr3.ReadLine();
                poss[i * 3 + 1] = (float)Convert.ToDouble(line);
                line = sr3.ReadLine();
                poss[i * 3 + 2] = (float)Convert.ToDouble(line);

                line = sr3.ReadLine();
                vels[i * 3] = (float)Convert.ToDouble(line);
                line = sr3.ReadLine();
                vels[i * 3 + 1] = (float)Convert.ToDouble(line);
                line = sr3.ReadLine();
                vels[i * 3 + 2] = (float)Convert.ToDouble(line);
            }
            Update(dt, agentNum, ids, poss, vels);
            count++;
            Clear();
        }
    }
}
