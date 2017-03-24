using System;
using System.Collections.Generic;
using System.Drawing;
using Rhino.Geometry;
using Grasshopper;
using Plankton;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using K2Collisions.CustomGoals;
using K2Collisions.DataStructures;
using KangarooSolver;
using PlanktonGh;

namespace K2Collisions.CustomGoals_Components
{
    public class HingeAll_Component : GH_Component
    {
        //Meshes
        Mesh mesh; //Input mesh is joined clean mesh
        PlanktonMesh[] pm; //Input mesh is disjoint into array of meshes
        DataTree<int> ne = new DataTree<int>();
        DataTree<int> ce = new DataTree<int>();
        DataTree<int> v = new DataTree<int>();

        List<IGoal> goals = new List<IGoal>();

        public HingeAll_Component() :base("K2 Plankton", "K2 PL GH", "K2 solver for Plankton mesh for LineLine collisions", "Kangaroo2", "K2 Custom Goals")
        {
            
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("{a400d67f-05d2-4a13-8dbc-240173c457e6}");}
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Angle", "A", "Angle", GH_ParamAccess.item,0);
            pManager.AddNumberParameter("Strength", "S", "Strenght", GH_ParamAccess.item,1);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Goal", "G", "Goal", GH_ParamAccess.item);

        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double angle = 0;
            double strength = 1;
            Mesh m = null;
            if (!DA.GetData(0, ref m)) return;
            DA.GetData(1, ref angle);
            DA.GetData(2, ref strength);

            mesh = m.DuplicateMesh();


            Init();

            DA.SetData(0, hingeGoals2(angle, strength));
        }

        private void Init()
        {
            //Disjoint meshes
            Mesh[] tempMeshes = mesh.SplitDisjointPieces();
            pm = new PlanktonMesh[tempMeshes.Length];

            //Naked edges and vertives
            ne = new DataTree<int>();
            ce = new DataTree<int>();
            v = new DataTree<int>();

            for (int i = 0; i < pm.Length; i++)
            {
                //Convert to pm
                pm[i] = tempMeshes[i].ToPlanktonMesh();

                //Naked edges
                for (int j = 0; j < pm[i].Halfedges.Count; j += 2)
                    if (pm[i].Halfedges.IsBoundary(j)) 
                        ne.Add(j, new GH_Path(i));
                    else
                        ce.Add(j, new GH_Path(i));


                //Vertices
                for (int j = 0; j < pm[i].Vertices.Count; j++)
                    v.Add(j, new GH_Path(i));

            }
        }

        private IGoal hingeGoals(double a, double s)
        {
            int[] IEdgeEnd1 = new int[ce.AllData().Count];
            int[] IEdgeEnd2 = new int[ce.AllData().Count];
            int[] ITip1 = new int[ce.AllData().Count];
            int[] ITip2 = new int[ce.AllData().Count];


            int count = 0;
            int n = 0;



            for (int i = 0; i < pm.Length; i++)
            {
                for (int j = 0; j < ce.Branch(i).Count; j++)
                {
                    IEdgeEnd1[n] = pm[i].Halfedges[ce.Branch(i)[j]].StartVertex + count;
                    IEdgeEnd2[n] = pm[i].Halfedges[ce.Branch(i)[j] + 1].StartVertex + count;
                    int prev1 = pm[i].Halfedges[ce.Branch(i)[j]].PrevHalfedge;
                    int prev2 = pm[i].Halfedges[ce.Branch(i)[j] + 1].PrevHalfedge;

                    ITip1[n] = pm[i].Halfedges[prev1].StartVertex + count;
                    ITip2[n] = pm[i].Halfedges[prev2].StartVertex + count;
                    n++;
                }
                count += pm[i].Vertices.Count;
            }

            return new HingeAll(IEdgeEnd1, IEdgeEnd2, ITip2, ITip1, a, s);
        }


        private IGoal hingeGoals2(double a, double s)
        {
            //Topology vertex indices
            List<int> IEdgeEnd1 = new List<int>();
            List<int> IEdgeEnd2 = new List<int>();
            List<int> ITip1 = new List<int>();
            List<int> ITip2 = new List<int>();

            int n = 0;

            foreach (PlanktonMesh planktonMesh in pm)
            {
                for (int i = 0; i < planktonMesh.Halfedges.Count; i = i + 2)
                {
                    if (planktonMesh.Halfedges.IsBoundary(i)) continue;
                    IEdgeEnd1.Add(planktonMesh.Halfedges[i].StartVertex + n);
                    IEdgeEnd2.Add(planktonMesh.Halfedges[i + 1].StartVertex + n);
                    ITip1.Add(planktonMesh.Halfedges[planktonMesh.Halfedges[i].PrevHalfedge].StartVertex + n);
                    ITip2.Add(planktonMesh.Halfedges[planktonMesh.Halfedges[i + 1].PrevHalfedge].StartVertex + n);
                }
                n += planktonMesh.Vertices.Count;
            }

            return new CustomGoals.HingeAll(IEdgeEnd1.ToArray(), IEdgeEnd2.ToArray(), ITip1.ToArray(), ITip2.ToArray(), a, s);
            // goals.Add(new KangarooSolver.Goals.HingeAll(IEdgeEnd1.ToArray(), IEdgeEnd2.ToArray(), ITip1.ToArray(), ITip2.ToArray(), 0, goalValues[3]));
        }

        protected override Bitmap Icon
        {
            get { return base.Icon; }
        }
    }
}
