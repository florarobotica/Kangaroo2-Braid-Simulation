using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel;

namespace K2Collisions.Utils
{
    public class GoalParametersComponent : GH_Component
    {
        public override Guid ComponentGuid
        {
            get { return  new Guid("ea12967b-1931-4388-83c2-da264f390dab"); }
        }


        public GoalParametersComponent() : base("Values", "Values",
              "Values",
              "Kangaroo2", "K2Weaver")
        {
            
        }

        protected override Bitmap Icon
        {
            get { return Properties.Resources.values; }
        }

        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.secondary; }
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("Clothed Edges", "Clothed Edges", "Clothed Edges", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Naked Edges", "Naked Edges", "Naked Edges", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Springs Strength", "Spring Strength", "Springs Strength", GH_ParamAccess.item, 10.0);
            pManager.AddNumberParameter("Hinge Strength", "Hinge Strength", "Hinge Strength", GH_ParamAccess.item, 0.5);
            pManager.AddNumberParameter("Collision Radius", "Collision Radius", "Collision Radius", GH_ParamAccess.item, 0.05);
            pManager.AddNumberParameter("Collision Strength", "Collision Strength", "Collision Strength", GH_ParamAccess.item, 100.0);
            pManager.AddNumberParameter("Offset SpatialHash", "Offset SpatialHash", "Offset SpatialHash", GH_ParamAccess.item, 1.0);

            for (int i = 0; i < pManager.ParamCount; i++)
                pManager[i].Optional = true;


        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Values", "V", "Values", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double[] values = new double[7];

            for (int i = 0; i < 7; i++)
                DA.GetData(i, ref values[i]);

            DA.SetDataList(0, values.ToArray());

        }
    }
}
