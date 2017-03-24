using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using Grasshopper.Kernel;
using KangarooSolver;
using K2Collisions.CustomGoals;
using Rhino.Geometry;
using PolyMesh_Core.Geometry;
using PolyMesh_Common;
using K2Collisions.DataStructures;



namespace K2Collisions.Solvers
{
    public class K2CollisionsPolylmeshComponent : GH_Component
    {
        private Mesh mesh;
        private PhysicalSystem PS = new PhysicalSystem();
        private bool Running;
        private bool anchors;

        private List<KangarooSolver.IGoal> goals = new List<KangarooSolver.IGoal>();

        private PolyMesh pMesh; //Input mesh is disjoint into array of meshes

        private int NCollisions = 0; //Number of linelineCollisions

        //Spatial Grid
        private Box box;
        private SpatialGrid3d<int> _grid; //Search methdod A
        private Domain3d _domain;
        private Vector3d subdivision;
        private Vec3d[] _positions;

        private int[][] _nakedEdgesVertices; //Ordered by loops
        private List<int> _nakedEdgesIndices = new List<int>();
        private List<int> _meshIndex = new List<int>(); 
        private int[] _clothedEdgesFlattened; //one flattened array of particle pair indices

        //No timer
        private bool stepType;

        protected override Bitmap Icon
        {
            get { return Properties.Resources.Solver2; }
        }

        public override Guid ComponentGuid { get { return new Guid("{ccc263df-3388-4886-a74f-511176bd0afc}"); } }

        public K2CollisionsPolylmeshComponent()
          : base("K2 Polymesh", "K2 PL",
              "K2 solver for Plankton mesh for LineLine collisions",
              "Kangaroo2", "K2Weaver")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
        
            pManager.AddBooleanParameter("Reset", "Reset", "Reset", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("On", "On", "On", GH_ParamAccess.item, false);
            pManager.AddParameter(new Param_PolyMesh());
            pManager.AddNumberParameter("Values", "Values", "Values", GH_ParamAccess.list, new List<double> { 1.2, 0.5, 10, 3, 0.075,100,2.5 });
            pManager.AddBoxParameter("BoundingBox", "BoundingBox", "BoundingBox", GH_ParamAccess.item);
            pManager.AddVectorParameter("Subdivisions", "Subdivisions", "Subdivisions", GH_ParamAccess.item, new Vector3d(10, 10, 10));
            pManager.AddBooleanParameter("Threading", "Threading", "Threading", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Valence x<4 Anchors", "Valence x<4 Anchors", "Valence x<4 Anchors", GH_ParamAccess.item, false);
        }


        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.primary; }
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
        }


        protected override void SolveInstance(IGH_DataAccess DA)
        {

            //Input
            bool flag = false;
            List<double> goalValues = new List<double>();
            //Length, Strength, StrengthHinge, StrengthLineLine, radiusRTree, radius

            DA.GetData(0, ref flag);
            DA.GetData(1, ref this.Running);
            if (!DA.GetData<PolyMesh>(2, ref pMesh)) return;
            DA.GetDataList(3, goalValues);
            if (!DA.GetData(4, ref box)) return;
            DA.GetData(5, ref subdivision);
            DA.GetData(6, ref this.stepType);
            DA.GetData(7, ref anchors);

            //Init
            if (PS.GetIterations() == 0 || flag)
            {
                Init(goalValues);
                anchorsGoals(anchors);
                springsGoals(goalValues);
              // angleGoals(goalValues);
                hingeGoals(goalValues);
                NCollisions = this.goals.Count;
                //SpatialGridCollisions(goalValues);
            }

            //Recalculate collisions
            SpatialGridCollisions(goalValues);


            //Solver UI
            if (Running && !flag)
            {
                if (this.stepType)
                    this.PS.Step(goals, true, 1E-15);
                else
                    this.PS.SimpleStep(goals);
            }


            //Locator
            for(int i = 0; i < pMesh.Vertices.Count; i++)
                pMesh.Vertices[i] = PS.GetPosition(i);

            //Output
            //DA.SetData(0, new GH_PolyMesh(pMesh) );
            mesh = pMesh.ToMesh(true);
            mesh.Normals.ComputeNormals();


        }

        private void Init(List<double> goalValues)
        {

            //Init KangarooSolver
            PS = new KangarooSolver.PhysicalSystem();
            goals.Clear();

            for (int i = 0; i < pMesh.Vertices.Count; i++)
                PS.AddParticle(pMesh.Vertices[i],1);
            
            //Get clothed and ordered naked edges
            var tops = pMesh.GetTopologyEdges();
            var eds = new List<Polyline>();
            var ted = new List<TopologyEdge>();
            var tedI = new List<int>();
            var nakedEdges = new List<int>();
            var tedClothed = new List<TopologyEdge>();
            var tedClothedFlattened = new List<int>();

            //SpatialGrid
            _positions = new Vec3d[pMesh.GetNakedEdges().Length];
            _domain = box.BoundingBox.ToDomain3d();
            _grid = new SpatialGrid3d<int>(_domain, (int)subdivision.X, (int)subdivision.Y, (int)subdivision.Z);


            for (int i = 0; i < tops.Count; i++)
            {
                if (tops[i].IsNaked)
                {
                    eds.Add(new Polyline(new List<Point3d> { pMesh.Vertices[tops[i].From], pMesh.Vertices[tops[i].To] }));
                    ted.Add(tops[i]);
                    tedI.Add(i);
                    _nakedEdgesIndices.Add(i);
                }
                else
                {
                    tedClothedFlattened.Add(tops[i].From);
                    tedClothedFlattened.Add(tops[i].To);
                }
            }

            var gr = new List<List<int>>();
            PolyMesh_Core.CommonTools.Utils.JoinPolylines(eds, ref gr);

            //1.1 Clothed edges in flattened list
            _clothedEdgesFlattened = tedClothedFlattened.ToArray();

            //1.2 Selecting naked topEdges
            _nakedEdgesVertices = new int[gr.Count][];
            _nakedEdgesIndices = new List<int>();
            _meshIndex = new List<int>();

            for(int i = 0; i < gr.Count; i++)
            {
                var temp = new List<TopologyEdge>();
                //Re-ordering naked edges indices
                foreach (var j in gr[i])
                {
                    temp.Add(ted[j]);
                    _nakedEdgesIndices.Add(tedI[j]);
                    _meshIndex.Add(i);
                }

                _nakedEdgesVertices[i] = orderedNakedIndices(temp);
            }


            //goals.Add(new LineLineColliderSpatialHash(_nakedVertices.ToArray(), edgesFlattened, _edges,  _grid, meshIndex, goalValues[4], goalValues[5], true));
            
        }

        public int[] orderedNakedIndices(List<TopologyEdge> x)
        {

            var ted = new List<TopologyEdge>();

            for (int i = 0; i < x.Count; i++)
                ted.Add((TopologyEdge)x[i]);

            var ind = new List<int>();
            ind.Add(ted[0].From);
            ind.Add(ted[0].To);



            if (ted[1].From != ind[1] && ted[1].To != ind[1])
            {
                int temp = ind[0];
                ind[0] = ind[1];
                ind[1] = temp;
            }


            for (int i = 1; i < ted.Count; i++)
            {
                TopologyEdge thise = ted[i];
                int pre1 = ind[ind.Count - 1];
                int pre2 = ind[ind.Count - 2];

                if (thise.From != pre1 && thise.From != pre2)
                    ind.Add(thise.From);
                else if (thise.To != pre1 && thise.To != pre2)
                    ind.Add(thise.To);

            }

            return ind.ToArray();
        }


        /// <summary>
        /// Spring goals
        /// </summary>
        /// <param name="goalValues"></param>
        private void springsGoals(List<double> goalValues)
        {
            for (int i = 0; i < _clothedEdgesFlattened.Length; i += 2)
                //goals.Add(new Spring(_clothedEdgesFlattened[i], _clothedEdgesFlattened[i + 1], goalValues[0], goalValues[2]));
                 goals.Add(new K2Collisions.CustomGoals.SpringGrow(_clothedEdgesFlattened[i], _clothedEdgesFlattened[i + 1], PS.GetPosition(_clothedEdgesFlattened[i]).DistanceTo(PS.GetPosition(_clothedEdgesFlattened[i + 1])), goalValues[0], goalValues[2]));

            foreach (var array in _nakedEdgesVertices)
                for (int i = 0; i < array.Length - 1; i ++)
                        //goals.Add(new Spring(array[i], array[i + 1], goalValues[1], goalValues[2]));

                            goals.Add(new K2Collisions.CustomGoals.SpringGrow(array[i], array[i + 1], PS.GetPosition(array[i]).DistanceTo(PS.GetPosition(array[i + 1])), goalValues[1], goalValues[2]));

            //Diagonals
            //for (int i = 0; i < pMesh.Faces.Count; i++)
            //{
            //    int[] array = pMesh.Faces[i].ToArray();

            //    goals.Add(new SpringGrow(array[0], array[2], PS.GetPosition(array[0]).DistanceTo(PS.GetPosition(array[2])), goalValues[1] * 1.41421356237, goalValues[2]));
            //    goals.Add(new SpringGrow(array[1], array[3], PS.GetPosition(array[1]).DistanceTo(PS.GetPosition(array[3])), goalValues[1] * 1.41421356237, goalValues[2]));
            //}


            //goals.Add(new SpringsAll(clothedAll.ToArray(), clothedAllDist.ToArray(), goalValues[1], goalValues[2]));
            //goals.Add(new SpringsAll(nakedAll.ToArray(), nakedAllDist.ToArray(), goalValues[1], goalValues[2]));
        }

        /// <summary>
        /// Anchors
        /// </summary>
        /// <param name=""></param>
        private void anchorsGoals(bool anchorsFlag)
        {
            if (anchorsFlag)
                for (int i = 0; i < pMesh.Vertices.Count; i++)
                    if (pMesh.Vertices.GetConnectedVertices(i).Count <= 2)
                        goals.Add(new KangarooSolver.Goals.Anchor(i, pMesh.Vertices[i], 1000000));
        }

        /// <summary>
        /// Angle goals
        /// </summary>
        /// <param name="goalValues"></param>
        private void angleGoals(List<double> goalValues)
        {
            foreach (var array in _nakedEdgesVertices)
            {
                for (int i = 0; i < array.Length - 2; i++)
                {
                    if (pMesh.Vertices.GetConnectedVertices(array[i + 1]).Count <= 3) continue; //No angle for corners
                        goals.Add(new KangarooSolver.Goals.Angle(goalValues[3], 0, array[i], array[i + 1], array[i + 1], array[i + 2]));
                }
                if (pMesh.Vertices.GetConnectedVertices(array[array.Length - 1]).Count <= 3) continue; //No angle for corners
                goals.Add(new KangarooSolver.Goals.Angle(goalValues[3], 0, array[array.Length-2], array[array.Length-1], array[array.Length-1],array[1]));
            }
        }

        private void hingeGoals(List<double> goalValues)
        {
            //Topology vertex indices
            List<int> IEdgeEnd1 = new List<int>();
            List<int> IEdgeEnd2 = new List<int>();
            List<int> ITip1 = new List<int>();
            List<int> ITip2 = new List<int>();

            var topEdges = pMesh.GetTopologyEdges();

            for (int i = 0; i< topEdges.Count; i++)
            {
                if (!topEdges[i].IsNaked)
                {
                    IEdgeEnd1.Add(topEdges[i].From);
                    IEdgeEnd2.Add(topEdges[i].To);

   
                    foreach (int id in pMesh.Faces[topEdges[i].LeftFace])
                        if (id != topEdges[i].From && id != topEdges[i].To)
                            ITip1.Add(id);

                    foreach (int id in pMesh.Faces[topEdges[i].RightFace])
                        if (id != topEdges[i].From && id != topEdges[i].To)
                            ITip2.Add(id);
                }
            }

            goals.Add(new K2Collisions.CustomGoals.HingeAll(IEdgeEnd1.ToArray(), IEdgeEnd2.ToArray(), ITip1.ToArray(), ITip2.ToArray(), 0, goalValues[3]));

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
            _positions = new Vec3d[_nakedEdgesIndices.Count];
            var top = pMesh.GetTopologyEdges();

            //Insert positions
            for (int i = 0; i < _positions.Length; i++)
                _positions[i] = ((p[top[_nakedEdgesIndices[i]].From] + p[top[_nakedEdgesIndices[i]].To])*0.5).ToVec3d();

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
                        if (j <= i ) continue;// _meshIndex[j] == _meshIndex[i]) continue;
                        a.Add(top[_nakedEdgesIndices[i]].From);
                        b.Add(top[_nakedEdgesIndices[i]].To);
                        c.Add(top[_nakedEdgesIndices[j]].From);
                        d.Add(top[_nakedEdgesIndices[j]].To);
                    }
                });
            }

            //Clear grid
            _grid.Clear();
            goals.Add(new LineLineCollider(a.ToArray(), b.ToArray(), c.ToArray(), d.ToArray(), goalValues[4], goalValues[5], true));
        }




        /// <summary>
        /// Timer
        /// </summary>
        protected override void AfterSolveInstance()
        {
            if (!this.Running)
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

        /// <summary>
        /// Display Lines
        /// </summary>
        /// <param name="args"></param>
        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
          //base.DrawViewportWires(args);
            //Display line
            //var lines = PS.GetLinesOutput(goals);
            //var springs = PS.GetLineOutput(goals);
        // args.Display.DrawLines(lines, Color.Black, 2);
         //   args.Display.DrawLines(springs, Color.Black, 1);
        }

        /// <summary>
        /// Display Mesh
        /// </summary>
        /// <param name="args"></param>
        public override void DrawViewportMeshes(IGH_PreviewArgs args)
        {
           // base.DrawViewportWires(args);
            var material = new Rhino.Display.DisplayMaterial(Color.FromArgb(255, 255, 255));
            material.IsTwoSided = true;
            material.BackDiffuse = Color.Red;

            if (pMesh != null)
            {
                args.Display.DrawMeshShaded(mesh, material);
                //Polyline[] p = pMesh.GetNakedEdges();

                //foreach (var pp in p)
                //    args.Display.DrawPolyline(pp, Color.Black, 2);
              // args.Display.DrawMeshWires(mesh, Color.Black);


            }

        }

    }
}
