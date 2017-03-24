using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using Plankton;
using PlanktonGh;
using Grasshopper.Kernel;
using KangarooSolver;
using KangarooSolver.Goals;
using Rhino.Geometry;
using K2Collisions.DataStructures;



namespace K2Collisions.Solvers
{
    public class K2CollisionsPlanktonComponent : GH_Component
    {

        #region global variables
        private PhysicalSystem PS = new PhysicalSystem();
        private bool Running;
        private double VSum;
        private bool anchors;

        private List<KangarooSolver.IGoal> goals = new List<KangarooSolver.IGoal>();

        private Mesh mesh; //Input mesh is joined clean mesh
        private PlanktonMesh[] meshes; //Input mesh is disjoint into array of meshes
        private int NCollisions = 0; //Number of linelineCollisions

        //Spatial Grid
        private Box box;
        private SpatialGrid3d<int> _grid; //Search methdod A
        private Domain3d _domain;
        private Vector3d subdivision;
        private Vec3d[] _positions;
        private int[][] _edges; //Edges mapped to particle indices
        private int[] edgesFlattened; //one flattened array of particle pair indices
        private int[] meshIndex; //for avoiding self collision calculation  
        private List<int> _nakedVertices = new List<int>(); //For line line collision

        //No timer
        private bool Threading;

        //Material
        Rhino.Display.DisplayMaterial material = new Rhino.Display.DisplayMaterial(Color.FromArgb(255, 255, 255));
        Mesh previewMesh = new Mesh();
        List<Line> naked = new List<Line>(); 

        #endregion

        #region icon and guid
        protected override Bitmap Icon
        {
            get { return Properties.Resources.Solver; }
        }

        public override Guid ComponentGuid { get { return new Guid("{ccc263df-3388-4886-a74e-511176bd7afc}"); } }
        #endregion

        #region timer
        protected override void AfterSolveInstance()
        {
            if (!this.Running || this.VSum < 1E-15)
            {
                return;
            }
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

        #region preview
        /// <summary>
        /// Display Lines
        /// </summary>
        /// <param name="args"></param>
        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            //Display line
            //var lines = PS.GetLinesOutput(goals);
            args.Display.DrawLines(naked, Color.Black, 1);
        }

        /// <summary>
        /// Display Mesh
        /// </summary>
        /// <param name="args"></param>
        public override void DrawViewportMeshes(IGH_PreviewArgs args)
        {
            if (previewMesh != null)
                    args.Display.DrawMeshShaded(previewMesh, material);

        }
        #endregion



        public K2CollisionsPlanktonComponent()
          : base("K2 Plankton", "K2 PL",
              "K2 solver for Plankton mesh for LineLine collisions",
              "Kangaroo2", "K2Weaver")
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
        }

        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.primary; }
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            material.IsTwoSided = true;
            material.BackDiffuse = Color.Red;
            //Input
            bool flag = false;
            List<double> goalValues = new List<double>();

            //Length, Strength, StrengthHinge, StrengthLineLine, radiusRTree, radius

            DA.GetData(0, ref flag);
            DA.GetData(1, ref this.Running);
            if (!DA.GetData(2, ref mesh)) return;
            DA.GetDataList(3, goalValues);
            if (!DA.GetData(4, ref box)) return;
            DA.GetData(5, ref subdivision);
            DA.GetData(6, ref this.Threading);
            DA.GetData(7, ref anchors);

            //Init
            if (PS.GetIterations() == 0 || flag)
                Init(goalValues);

            //Recalculate collisions
            SpatialGridCollisions(goalValues);


            //Solver UI
            if (flag)
            {
                this.Running = false;
                this.PS.Restart();

            }
            else if (Threading)
            {
                this.PS.Step(goals, true, 1E-15);
                this.VSum = this.PS.GetvSum();
            }
            else
            {
                this.PS.SimpleStep(goals);
                //radians += 0.0025;
            }

            //Locator
            pseudoLocator();


            previewMesh = new Mesh();
            naked = new List<Line>();
            //Preview
            for (int i = 0; i < meshes.Length; i++)
            {
                

                Mesh temp = RhinoSupport.ToRhinoMesh(meshes[i]);
                previewMesh.Append((temp));
                for (int j = 0; j < temp.TopologyEdges.Count; j++)
                    if (temp.TopologyEdges.GetConnectedFaces(j).Length == 1)
                        naked.Add(temp.TopologyEdges.EdgeLine(j));
            }

        }

        /// <summary>
        /// LineLine Collisions
        /// </summary>
        /// <param name="goalValues"></param>
        private void SpatialGridCollisions(List<double> goalValues)
        {
            //Store particles that are influenced by collision
            List<int> a = new List<int>();
            List<int> b = new List<int>();
            List<int> c = new List<int>();
            List<int> d = new List<int>();

            //Remove previous collisions
            if (goals.Count - NCollisions > 0)
                goals.RemoveRange(NCollisions, goals.Count - NCollisions);

            //Get current positions
            Point3d[] p = PS.GetPositions().ToArray();

            //Insert positions
            //p = all current particles
            //_edges = edge array 0 1 2 3 4 5 6 edge index; where [][0] [][1] are vertex indices
            for (int i = 0; i < _positions.Length; i++)
                _positions[i] = ((p[_edges[i][0]] + p[_edges[i][1]]) * 0.5).ToVec3d();

            //Insert particles into a grid
            for (int i = 0; i < _positions.Length; i++)
                _grid.Insert(_positions[i], i);



            //Search
            double offset = goalValues[6];
            for (int i = 0; i < _positions.Length; i++)
            {
                Domain3d bbox = new Domain3d(_positions[i] - new Vec3d(0.1 * 2.0, 0.1 * 2.0, 0.1 * 2.0) * offset, _positions[i] + new Vec3d(0.1 * 2.0, 0.1 * 2.0, 0.1 * 2.0) * offset); //Search radius

                _grid.Search(bbox, foundIds =>
                {
                    //Rhino.RhinoApp.WriteLine(foundIds.Count().ToString());
                    foreach (int j in foundIds)
                    {
                        //Rhino.RhinoApp.WriteLine(j.ToString());
                        if (j <= i) continue;//
                        //if ( _meshIndex[j] == _meshIndex[i]) continue;
                        a.Add(_edges[i][0]);
                        b.Add(_edges[i][1]);
                        c.Add(_edges[j][0]);
                        d.Add(_edges[j][1]);
                    }
                });
            }

            //Clear grid
            _grid.Clear();
            goals.Add(new CustomGoals.LineLineCollider(a.ToArray(), b.ToArray(), c.ToArray(), d.ToArray(), goalValues[4], goalValues[5], true));

        }

        private void Init(List<double> goalValues)
        {
            //Init KangarooSolver
            PS = new KangarooSolver.PhysicalSystem();
            goals.Clear();

            Mesh[] tempMeshes = mesh.SplitDisjointPieces();
            meshes = new PlanktonMesh[tempMeshes.Length];

            //Calculate how many naked edge do we have
            int numberOfNakedEdges = 0;
            for (int j = 0; j < tempMeshes.Length; j++)
            {
                PlanktonMesh p = tempMeshes[j].ToPlanktonMesh();
                for (int i = 0; i < p.Halfedges.Count; i += 2)
                    if (p.Halfedges.IsBoundary(i))
                        numberOfNakedEdges++;
            }

            _edges = new int[numberOfNakedEdges][];
            edgesFlattened = new int[numberOfNakedEdges * 2];
            meshIndex = new int[numberOfNakedEdges];



            //Init grid
            _domain = box.BoundingBox.ToDomain3d();
            _grid = new SpatialGrid3d<int>(_domain, (int)subdivision.X, (int)subdivision.Y, (int)subdivision.Z);
            _positions = new Vec3d[numberOfNakedEdges]; //For SpatialGrid Search
            _nakedVertices = new List<int>();

            //Add particles using Plankton
            int e = 0; //tracking vertices 0..n
            int t = 0; //tracking mesh.Vertices.Count
            for (int j = 0; j < tempMeshes.Length; j++)
            {
                this.meshes[j] = RhinoSupport.ToPlanktonMesh(tempMeshes[j]); //Convert to Plankton Mesh

                //Add Vertices to Particle System
                for (int i = 0; i < this.meshes[j].Vertices.Count; i++)
                    PS.AddParticle(RhinoSupport.ToPoint3d(this.meshes[j].Vertices[i]), 1);


                //Add HalfEdge middle points for SpatialGrid Search

                for (int i = 0; i < this.meshes[j].Halfedges.Count; i += 2)
                {
                    if (meshes[j].Halfedges.IsBoundary(i))
                    {
                        int a = this.meshes[j].Halfedges[i].StartVertex;
                        int b = this.meshes[j].Halfedges.EndVertex(i);
                        _edges[e] = new int[] { t + a, t + b };
                        edgesFlattened[2 * e] = t + a;
                        edgesFlattened[2 * e + 1] = t + b;
                        meshIndex[e] = j;
                        e++;
                    }
                }

                // get Naked vertices
                for (int i = 0; i < this.meshes[j].Vertices.Count; i++)
                    if (this.meshes[j].Vertices.IsBoundary(j))
                        _nakedVertices.Add(i + t);

                //Anchors
                if (anchors)
                    for (int i = 0; i < this.meshes[j].Vertices.Count; i++)
                        if (meshes[j].Vertices.GetVertexNeighbours(i).Length <= 3)
                            goals.Add(new KangarooSolver.Goals.Anchor(i + t, meshes[j].Vertices[i].ToPoint3d(), 1000000));


                t += meshes[j].Vertices.Count;
            }

            //Add Goals
            springsGoals(goalValues);
            hingeGoals(goalValues);
            //goals.Add(new LineLineColliderSpatialHash(_nakedVertices.ToArray(), edgesFlattened, _edges,  _grid, meshIndex, goalValues[4], goalValues[5], true));
            NCollisions = this.goals.Count;
        }

        private void pseudoLocator()
        {
            //Make mesh preview from existing particles
            int loop = 0; //loop through all vertices
            for (int j = 0; j < meshes.Length; j++)
            {
                for (int i = 0; i < meshes[j].Vertices.Count; i++)
                    meshes[j].Vertices.SetVertex(i, PS.GetPosition(loop + i));
                loop += meshes[j].Vertices.Count;
            }
        }

        private void springsGoals(List<double> goalValues)
        {
            List<int> clothedS = new List<int>();
            List<int> nakedS = new List<int>();
            List<int> clothedE = new List<int>();
            List<int> nakedE = new List<int>();

            List<int> clothedAll = new List<int>();
            List<int> nakedAll = new List<int>();
            List<double> nakedAllDist = new List<double>();
            List<double> clothedAllDist = new List<double>();

            int n = 0;

            foreach (PlanktonMesh mesh in meshes)
            {
                var topEdges = mesh.Halfedges;

                for (var i = 0; i < topEdges.Count; i += 2)
                {
                    if (topEdges.IsBoundary(i))
                    {
                        nakedS.Add(topEdges[i].StartVertex + n);
                        nakedE.Add(topEdges[i + 1].StartVertex + n);

                        //nakedAll.Add(topEdges[i].StartVertex + n);
                        //nakedAll.Add(topEdges[i + 1].StartVertex + n);
                        //nakedAllDist.Add(PS.GetPosition(topEdges[i].StartVertex + n).DistanceTo(PS.GetPosition(topEdges[i + 1].StartVertex + n)));
                    }
                    else
                    {
                        clothedS.Add(topEdges[i].StartVertex + n);
                        clothedE.Add(topEdges[i + 1].StartVertex + n);

                        //clothedAll.Add(topEdges[i].StartVertex + n);
                        //clothedAll.Add(topEdges[i + 1].StartVertex + n);
                        //clothedAllDist.Add(PS.GetPosition(topEdges[i].StartVertex + n).DistanceTo(PS.GetPosition(topEdges[i + 1].StartVertex + n)));
                    }
                }
                n += mesh.Vertices.Count;
            }

            for (int i = 0; i < clothedE.Count; i++)
                goals.Add(new CustomGoals.SpringGrow(clothedS[i], clothedE[i], PS.GetPosition(clothedS[i]).DistanceTo(PS.GetPosition(clothedE[i])), goalValues[0], goalValues[2]));

            for (int i = 0; i < nakedS.Count; i++)
                goals.Add(new CustomGoals.SpringGrow(nakedS[i], nakedE[i], PS.GetPosition(nakedS[i]).DistanceTo(PS.GetPosition(nakedE[i])), goalValues[1], goalValues[2]));


            //goals.Add(new SpringsAll(clothedAll.ToArray(), clothedAllDist.ToArray(), goalValues[1], goalValues[2]));
            //goals.Add(new SpringsAll(nakedAll.ToArray(), nakedAllDist.ToArray(), goalValues[1], goalValues[2]));

        }

        private void hingeGoals(List<double> goalValues)
        {
            //Topology vertex indices
            List<int> IEdgeEnd1 = new List<int>();
            List<int> IEdgeEnd2 = new List<int>();
            List<int> ITip1 = new List<int>();
            List<int> ITip2 = new List<int>();

            int n = 0;

            foreach (PlanktonMesh planktonMesh in meshes)
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
        
           goals.Add((new CustomGoals.HingeAll(IEdgeEnd1.ToArray(), IEdgeEnd2.ToArray(), ITip1.ToArray(), ITip2.ToArray(), 0, goalValues[3])));
           // goals.Add(new KangarooSolver.Goals.HingeAll(IEdgeEnd1.ToArray(), IEdgeEnd2.ToArray(), ITip1.ToArray(), ITip2.ToArray(), 0, goalValues[3]));
        }
    }

}

