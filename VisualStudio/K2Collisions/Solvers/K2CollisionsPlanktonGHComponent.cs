using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using Grasshopper;
using Plankton;
using PlanktonGh;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using K2Collisions.CustomGoals;
using K2Collisions.DataStructures;
using Rhino.Geometry;
using KangarooSolver;
using KangarooSolver.Goals;

namespace K2Collisions.Solvers
{
    public class K2CollisionsPlanktonGHComponent : GH_Component
    {
        //Kangaroo
        PhysicalSystem PS = new PhysicalSystem();
        List<IGoal> goals = new List<IGoal>();
        List<double> goalValues = new List<double>();
        bool anchors;

        //Meshes
        Mesh mesh; //Input mesh is joined clean mesh
        PlanktonMesh[] pm; //Input mesh is disjoint into array of meshes
        DataTree<int> ne = new DataTree<int>();
        DataTree<int> ce = new DataTree<int>();
        List<int[]> nep = new List<int[]>();
        DataTree<int> v = new DataTree<int>();

        //SpatialGrid
        SpatialGrid3d<int> grid;
        Box box;
        Domain3d domain;
        Vector3d subdivision;
        Vec3d[] positions;
        private int NCollisions;


        //Animation
        bool Threading;
        bool Running;
        //double VSum;

        //Material
        Rhino.Display.DisplayMaterial material = new Rhino.Display.DisplayMaterial(Color.FromArgb(255, 255, 255));
        List<Line> naked = new List<Line>();

        #region input/output

        public K2CollisionsPlanktonGHComponent()
          : base("K2 Plankton", "K2 PL GH", "K2 solver for Plankton mesh for LineLine collisions", "Kangaroo2", "K2Weaver")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Reset", "Reset", "Reset", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("On", "On", "On", GH_ParamAccess.item, false);
            pManager.AddMeshParameter("Mesh", "Mesh", "Mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Values", "Values", "Values", GH_ParamAccess.list, new List<double> { 0.4, 0.2, 10, 100 });
            pManager.AddBoxParameter("BoundingBox", "BoundingBox", "BoundingBox", GH_ParamAccess.item);
            pManager.AddVectorParameter("Subdivisions", "Subdivisions", "Subdivisions", GH_ParamAccess.item, new Vector3d(10, 10, 10));
            pManager.AddBooleanParameter("Threading", "Threading", "Threading", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Valence x<4 Anchors", "Valence x<4 Anchors", "Valence x<4 Anchors", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Mesh", "M", "Mesh", GH_ParamAccess.list);
            pManager.AddGenericParameter("Positions", "P", "Positions", GH_ParamAccess.list);
        }

        #endregion

        protected override void SolveInstance(IGH_DataAccess DA)
        {

            //Input
            bool flag = false;
            List<double> values = new List<double>();  //Length, Strength, StrengthHinge, StrengthLineLine, radiusRTree, radius


            DA.GetData(0, ref flag);
            DA.GetData(1, ref Running);
            if (!DA.GetData(2, ref mesh)) return;
            DA.GetDataList(3, values);
            if (!DA.GetData(4, ref box)) return;
            DA.GetData(5, ref subdivision);
            DA.GetData(6, ref Threading);
            DA.GetData(7, ref anchors);

            goalValues = values;

            //Init
            if (PS.GetIterations() == 0 || flag)
            {
                Init(goalValues);
                springGoals();
                hingeGoals();
                //Here I am adding one single goal and handling collisions within goal 
              //  goals.Add(new LineLineAll(goalValues[4], goalValues[5], true,box,subdivision,domain,positions,nep, grid, PS.ParticleCount()));
            
                //Back again in counting and removing goals each frames and adding them each step
                NCollisions = goals.Count;
            }
          collisions();

            //Solver UI
            if (flag)
            {
                Running = false;
                PS.Restart();
                Running = true;
            }

            if (Threading && Running)
            {

                PS.Step(goals, true, 1E-15);
                Message = PS.GetIterations().ToString() + "Threading";

            }
            else if (!Threading && Running)
            {
                PS.SimpleStep(goals);
                Message = PS.GetIterations().ToString() + "Simple";
            }


            //Locator
            int count = 0;
            for (int i = 0; i < v.BranchCount; i++)
            {
                for (int j = 0; j < v.Branch(i).Count; j++)
                    pm[i].Vertices.SetVertex(v.Branch(i)[j], PS.GetPosition(count + j));
                count += v.Branch(i).Count;
            }

            DA.SetDataList(0, pm);
            DA.SetDataList(1, PS.GetOutput( new List<IGoal> { goals.Last() } ));

            //Display
            naked = new List<Line>();
            material.IsTwoSided = true;
            material.BackDiffuse = Color.Red;

            mesh = new Mesh();
            foreach (var p in pm)
                mesh.Append(p.ToRhinoMesh());







        }

        #region init
        private void Init(List<double> goalValues)
        {
            //Init KangarooSolver
            PS = new PhysicalSystem();
            goals.Clear();


            //Disjoint meshes
            Mesh[] tempMeshes = mesh.SplitDisjointPieces();
            pm = new PlanktonMesh[tempMeshes.Length];

            //Naked edges and vertives
            ne = new DataTree<int>();
            ce = new DataTree<int>();
            v = new DataTree<int>();
            nep = new List<int[]>(); //Edge particles in a flattened order 

            //neParticles.Add(new int[2] { pm[i].Halfedges.EndVertex(j) + counter, pm[i].Halfedges[j].StartVertex + counter }, new GH_Path(i));


            this.Message = pm.Length.ToString() + " stripes";

            int counter = 0;
            for (int i = 0; i < pm.Length; i++)
            {
                //Convert to pm
                pm[i] = tempMeshes[i].ToPlanktonMesh();

                //Naked edges
                for (int j = 0; j < pm[i].Halfedges.Count; j += 2)
                    if (pm[i].Halfedges.IsBoundary(j))
                    {
                        ne.Add(j, new GH_Path(i));
                        nep.Add(new int[2] { pm[i].Halfedges.EndVertex(j) + counter, pm[i].Halfedges[j].StartVertex + counter });
                    }
                    else
                        ce.Add(j, new GH_Path(i));

                //Vertices
                for (int j = 0; j < pm[i].Vertices.Count; j++)
                {
                    PS.AddParticle(RhinoSupport.ToPoint3d(pm[i].Vertices[j]), 1);
                    v.Add(j, new GH_Path(i));
                }

                counter += pm[i].Vertices.Count;
            }


            //Spatial Grid
            domain = box.BoundingBox.ToDomain3d();
            grid = new SpatialGrid3d<int>(domain, (int)subdivision.X, (int)subdivision.Y, (int)subdivision.Z);
            positions = new Vec3d[ne.DataCount]; //Equals  to naked edges because we are checking only middle point


            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //Goals

            //Anchors
            int count = 0;
            int n = 0;
            if (anchors)
                for (int i = 0; i < pm.Length; i++)
                {
                    for (int j = 0; j < pm[i].Vertices.Count; j++)
                    {
                        //goals.Add(new Unary(n,Vector3d.ZAxis*0.12));
                        n++;
                        if (pm[i].Vertices.GetVertexNeighbours(j).Length <= 3 )//|| i % 9 == 0)
                            //goals.Add(new PlasticNails(v.Branch(i)[j]+count, pm[i].Vertices[j].ToPoint3d(), goalValues[1]*10,100));
                            goals.Add(new XYZNails(v.Branch(i)[j] + count, pm[i].Vertices[j].ToPoint3d(), true, true,
                                true, 10000));

                    }
                    count += pm[i].Vertices.Count;
                }



        }
        #endregion

        #region springs
        private void springGoals()
        {

            int count = 0;

            //Loop through each mesh
            for (int i = 0; i < pm.Length; i++)
            {

                List<int> corners = new List<int>();

                //Get Halfedges of two.
                for (int j = 0; j < pm[i].Vertices.Count; j++)
                    if (pm[i].Vertices.GetHalfedges(j).Length == 2)
                        corners.Add(j);

                //Clothed
                for (int j = 0; j < ce.Branch(i).Count; j++)
                {
                    int s = pm[i].Halfedges[ce.Branch(i)[j]].StartVertex; //Start index in nested array
                    int e = pm[i].Halfedges[ce.Branch(i)[j] + 1].StartVertex;//End index in nested array
                    double dist = pm[i].Halfedges.GetLength(ce.Branch(i)[j]);//Current Distance
                    goals.Add(new SpringGrow(count + s, count + e, dist, goalValues[0], goalValues[2]));
                }

                //Naked
                for (int j = 0; j < ne.Branch(i).Count; j++)
                {
                    int s = pm[i].Halfedges[ne.Branch(i)[j]].StartVertex; //Start index in nested array
                    int e = pm[i].Halfedges[ne.Branch(i)[j] + 1].StartVertex;//End index in nested array
                    double dist = pm[i].Halfedges.GetLength(ne.Branch(i)[j]);//Current Distance

                    int sum = pm[i].Vertices.GetHalfedges(s).Length + pm[i].Vertices.GetHalfedges(e).Length; //Check 


                    if (sum > 5)
                        goals.Add(new SpringGrow(count + s, count + e, dist, goalValues[1], goalValues[2]));
                    else
                        goals.Add(new SpringGrow(count + s, count + e, dist, goalValues[0], goalValues[2]));

                }

                count += pm[i].Vertices.Count;
            }

        }
        #endregion

        #region hinge To Do: Increase gradually. Wrap without indices
        private void hingeGoals()
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

            goals.Add(new HingeAll(IEdgeEnd1, IEdgeEnd2, ITip2, ITip1, 0, goalValues[3]));
        }
        #endregion

        #region collisions



        //Check this and remove all the use of Vector3D to Point3d
        private void collisions()
        {
            //Store particles that are influenced by collision
            List<int> a = new List<int>(v.DataCount);
            List<int> b = new List<int>(v.DataCount);
            List<int> c = new List<int>(v.DataCount);
            List<int> d = new List<int>(v.DataCount);

            //Remove previouse collisions; NCollisions are all goals except of collisions
            if (goals.Count - NCollisions > 0)
                goals.RemoveRange(NCollisions, goals.Count - NCollisions);


            // Get current edges middle points
            for (int i = 0; i < positions.Length; i++)
                positions[i] = ((PS.GetPosition(nep[i][0]) + PS.GetPosition(nep[i][1])) * 0.5).ToVec3d();


            //Insert particles into grid
            for (int i = 0; i < positions.Length; i++)
                grid.Insert(positions[i], i);

            //Search
            double offset = goalValues[6];
            for (int i = 0; i < positions.Length; i++)
            {
                Domain3d bbox = new Domain3d(positions[i] - new Vec3d(0.2, 0.2, 0.2) * offset, positions[i] + new Vec3d(0.2, 0.2, 0.2) * offset); //Search radius
                grid.Search(bbox, foundIds =>
                {
                    foreach (int j in foundIds)
                    {
                        if (j <= i) continue;

                        if (((PS.GetPosition(nep[i][0]) + PS.GetPosition(nep[i][1])) * 0.5).DistanceTo(  ((PS.GetPosition(nep[j][0]) + PS.GetPosition(nep[j][1])) * 0.5)) <  1)
                        {
                            //mapping lines middle points back to particles
                            a.Add(nep[i][0]);
                            b.Add(nep[i][1]);
                            c.Add(nep[j][0]);
                            d.Add(nep[j][1]);
                        }
                    }

                });
            }

            //Clear Grid
            grid.Clear();
            goals.Add(new LineLineCollider(a.ToArray(), b.ToArray(), c.ToArray(), d.ToArray(), goalValues[4], goalValues[5], true));
        }
        #endregion

        #region timer
        protected override void AfterSolveInstance()
        {
            if (!this.Running)
                return;

            GH_Document gH_Document = base.OnPingDocument();
            if (gH_Document == null)
            {
                return;
            }
            GH_Document.GH_ScheduleDelegate gH_ScheduleDelegate = new GH_Document.GH_ScheduleDelegate(this.ScheduleCallback);
            gH_Document.ScheduleSolution(1, gH_ScheduleDelegate);
        }

        private void ScheduleCallback(GH_Document doc)
        {
            this.ExpireSolution(false);
        }
        #endregion

        #region overrides
        protected override Bitmap Icon
        {
            get { return Properties.Resources.Solver; }
        }

        public override Guid ComponentGuid { get { return new Guid("{ccc589df-3388-4886-a14e-511176bd7afc}"); } }

        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.quinary; }
        }
        #endregion

        #region display

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            args.Display.DrawLines(naked, Color.Black, 1);
        }

        public override void DrawViewportMeshes(IGH_PreviewArgs args)
        {
            if (mesh != null)
                args.Display.DrawMeshShaded(mesh, material);
        }

        #endregion
    }
}
