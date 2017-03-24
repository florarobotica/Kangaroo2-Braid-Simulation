using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using PolyMesh_Core;
using PolyMesh_Core.Weaving;
using PolyMesh_Common;
using Rhino.Geometry;

using PolyMesh_Core.Geometry;

namespace K2Collisions.Weaver
{
    public class WeaverComponent : GH_Component
    {
        public override Guid ComponentGuid
        {
            get
            {
                Guid result = new Guid("c8f6c530-b0ce-4830-adb8-f75305708a8f");
                return result;
            }
        }
        protected override Bitmap Icon
        {
            get { return Properties.Resources.weaver; }
        }

        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.tertiary; }
        }

        public WeaverComponent() : base("Weaver", "Weaver", "Creates a weaving pattern.", "Kangaroo2", "K2Weaver")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_PolyMesh());
            pManager.AddIntegerParameter("Colors", "C", "Edge colors 0:Empty 1:Square 2:Diamond", GH_ParamAccess.list);
            pManager.AddNumberParameter("Depth", "D", "Weave depth", GH_ParamAccess.item, 0.1);
            pManager.AddNumberParameter("Triangle", "W", "Triangle width for triangulated mesh", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Type", "T", "Geometry type 0:Simple mesh 1:Triangulated mesh 2:Polylines", GH_ParamAccess.item, 0);
            pManager.AddParameter(new Param_TileDesign(), "Tiles", "#", "TileDesign", GH_ParamAccess.list);
            this.Params.Input[3].Optional = true;
            this.Params.Input[5].Optional = true;
            pManager.AddNumberParameter("W2", "W2", "W2", GH_ParamAccess.item, 0.1);
            this.Params.Input[6].Optional = true;
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Geometry", "G", "Geometry", GH_ParamAccess.tree);
            pManager.AddGenericParameter("Connections", "C", "Connections", GH_ParamAccess.tree);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool bothSides = true;
            PolyMesh polyMesh = null;
            bool flag = !DA.GetData<PolyMesh>(0, ref polyMesh);
            double w2 = 0.1;
            DA.GetData(6, ref w2);
            checked
            {
                if (!flag)
                {
                    List<int> list = new List<int>();
                    bool flag2 = !DA.GetDataList<int>(1, list);
                    if (!flag2)
                    {
                        double num = 0.1;
                        bool flag3 = !DA.GetData<double>(2, ref num);
                        if (!flag3)
                        {
                            bool flag4 = num < 0.0;
                            if (flag4)
                            {
                                num = Math.Abs(num);
                                bothSides = false;
                            }
                            List<double> list2 = new List<double>();
                            bool flag5 = !DA.GetDataList<double>(3, list2);
                            if (flag5)
                            {
                                list2 = new List<double>();
                                list2.Add(1.0);
                            }
                            bool flag6 = list2.Count == 1;
                            if (flag6)
                            {
                                int num2 = polyMesh.Faces.Count - 1;
                                for (int i = 1; i <= num2; i++)
                                {
                                    list2.Add(list2[0]);
                                }
                            }
                            else
                            {
                                bool flag7 = list2.Count != polyMesh.Faces.Count;
                                if (flag7)
                                {
                                    this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Face count <> Value count");
                                    return;
                                }
                            }
                            int num3 = 0;
                            bool flag8 = !DA.GetData<int>(4, ref num3);
                            if (!flag8)
                            {
                                List<TileDesign> list3 = new List<TileDesign>();
                                DA.GetDataList<TileDesign>(5, list3);
                                int num4 = polyMesh.Faces.Count - 1;
                                for (int j = 0; j <= num4; j++)
                                {
                                    bool flag9 = polyMesh.Faces[j].Count > 4 | polyMesh.Faces[j].Count < 3;
                                    if (flag9)
                                    {
                                        this.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Weaver can only work with triangles and quads.");
                                        break;
                                    }
                                }
                                List<TopologyEdge> topologyEdges = polyMesh.GetTopologyEdges();
                                PolyMesh_Core.Weaving.Weaver weaver = new PolyMesh_Core.Weaving.Weaver(polyMesh);
                                bool flag10 = list3.Count != 0;
                                if (flag10)
                                {
                                    int num5 = list3.Count - 1;
                                    for (int k = 0; k <= num5; k++)
                                    {
                                        weaver.TileDictionary[list3[k].Hash] = list3[k].Strips;
                                    }
                                }
                                SortedList<TopologyEdge, EdgeColor> sortedList = new SortedList<TopologyEdge, EdgeColor>();
                                int num6 = list.Count - 1;
                                for (int l = 0; l <= num6; l++)
                                {
                                    switch (list[l])
                                    {
                                        case 0:
                                            sortedList.Add(topologyEdges[l], EdgeColor.Empty);
                                            break;
                                        case 1:
                                            sortedList.Add(topologyEdges[l], EdgeColor.Plain);
                                            break;
                                        case 2:
                                            sortedList.Add(topologyEdges[l], EdgeColor.Braid);
                                            break;
                                    }
                                }
                                weaver.ApplyColors(sortedList);
                                GH_Structure<IGH_Goo> gH_Structure = new GH_Structure<IGH_Goo>();
                                switch (num3)
                                {
                                    case 0:
                                        DA.SetDataTree(0, this.ArrayToStructure(weaver.BuildSimpleMeshes(num, true, bothSides), DA.Iteration));
                                        return;
                                    case 1:
                                        DA.SetDataTree(0, this.ArrayToStructure(weaver.BuildTriangulatedMeshes(num, list2.ToArray(), true, bothSides), DA.Iteration));
                                        return;
                                    case 2:
                                        DA.SetDataTree(0, this.ArrayToStructure(weaver.BuildPolylines(num, true, bothSides), DA.Iteration));
                                        break;
                                    case 3:
                                        List<PolyMesh> ml = new List<PolyMesh>();

                                        for (int i = 0; i < polyMesh.Faces.Count; i++)
                                            ml.AddRange(weaver.BuildVariableMesh(i, num, list2[0], w2, true, bothSides));
                                        DA.SetDataTree(0, new DataTree<PolyMesh>(ml));
                                        break;
;

                                }
                            }
                        }
                    }
                }
            }
        }

        private GH_Structure<IGH_Goo> ArrayToStructure(List<PolyMesh>[] A, int pathprefix = 0)
        {
            GH_Structure<IGH_Goo> gH_Structure = new GH_Structure<IGH_Goo>();
            checked
            {
                int num = A.Length - 1;
                for (int i = 0; i <= num; i++)
                {
                    bool flag = A[i] == null;
                    if (!flag)
                    {
                        int num2 = A[i].Count - 1;
                        PolyMesh pMesh = new PolyMesh();
                        for (int j = 0; j <= num2; j++)
                            //gH_Structure.Append(new GH_PolyMesh(A[i][j]), new GH_Path(new int[] { pathprefix, i }));
                            pMesh.Append(A[i][j]);
                        gH_Structure.Append(new GH_PolyMesh(pMesh), new GH_Path(new int[] { pathprefix,i}));

                    }
                }
                return gH_Structure;
            }
        }

        private DataTree<int> ArrayToStructureEdges(List<PolyMesh>[] A, int pathprefix = 0)
        {
            DataTree<int> dt = new DataTree<int>();
            checked
            {
                int num = A.Length - 1;
                for (int i = 0; i <= num; i++)
                {
                    bool flag = A[i] == null;
                    if (!flag) 
                    {
                        int num2 = A[i].Count - 1;

                        PolyMesh pMesh = new PolyMesh();
                        
                        for (int j = 0; j <= num2; j++)
                            pMesh.Append(new PolyMesh(A[i][j]));


                        var e = pMesh.GetTopologyEdges();
                        for (int k = 0; k < e.Count; k++)
                            if (!e[k].IsNaked)
                            {
                                dt.Add(e[k].From, new GH_Path(new int[] { pathprefix, i }));
                                dt.Add(e[k].To, new GH_Path(new int[] { pathprefix, i }));
                            }


                    }
                }
                return dt;
            }
        }

        private GH_Structure<IGH_Goo> ArrayToStructure(List<Polyline>[] A, int pathprefix = 0)
        {
            GH_Structure<IGH_Goo> gH_Structure = new GH_Structure<IGH_Goo>();
            checked
            {
                int num = A.Length - 1;
                for (int i = 0; i <= num; i++)
                {
                    bool flag = A[i] == null;
                    if (!flag)
                    {
                        int num2 = A[i].Count - 1;
                        for (int j = 0; j <= num2; j++)
                        {
                            NurbsCurve nurbsCurve = A[i][j].ToNurbsCurve();
                            bool flag2 = nurbsCurve != null;
                            if (flag2)
                            {
                                gH_Structure.Append(new GH_Curve(nurbsCurve), new GH_Path(new int[]
                                {
                                    pathprefix,
                                    i
                                }));
                            }
                        }
                    }
                }
                return gH_Structure;
            }
        }
    }
}
