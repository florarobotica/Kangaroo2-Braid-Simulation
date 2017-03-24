using System;
using System.Drawing;
using Grasshopper.Kernel;
using Rhino.Geometry;
using K2Collisions.DataStructures;

namespace K2Collisions.Utils
{
    public class BoundingBoxComponent : GH_Component
    {
        public override Guid ComponentGuid
        {
            get { return new Guid("611c8b7e-d04d-4d87-a4f7-b57a7c21c174"); }
        }


        public BoundingBoxComponent() : base("Collision Box", "Collision Box",
              "Collision Box",
              "Kangaroo2", "K2Weaver")
        {

        }

        protected override Bitmap Icon
        {
            get { return Properties.Resources.box; }
        }

        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.secondary; }
        }


        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {

            pManager.AddMeshParameter("Mesh", "Mesh", "Mesh for bounding box", GH_ParamAccess.item);
            pManager.AddNumberParameter("Inflate", "I", "Inflate bounding box value", GH_ParamAccess.item);
            pManager.AddNumberParameter("Bin Scale", "Bin Scale", "Bin Scale (The bigger number the slower and more accurate calucation)", GH_ParamAccess.item);

            for (int i = 0; i < pManager.ParamCount; i++)
                pManager[i].Optional = true;


        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddBoxParameter("BoundingBox", "BBox","BoundingBox",GH_ParamAccess.item) ;
            pManager.AddVectorParameter("Subdivisions", "S", "Subdivisions as 3 doubles wrapped in one vector",GH_ParamAccess.item) ;
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {

            Mesh m = new Mesh();
            double y = 0;
            double binScale = 1;

            DA.GetData(0, ref m);
            DA.GetData(1, ref y);
            DA.GetData(2, ref binScale);


            BoundingBox bbox = m.GetBoundingBox(true);
            bbox.Inflate(y);


            Domain3d domain = bbox.ToDomain3d();
            Vec3d d = domain.Span;


            int nx = (int)Math.Ceiling(d.x / binScale);
            int ny = (int)Math.Ceiling(d.y / binScale);
            int nz = (int)Math.Ceiling(d.z / binScale);

            DA.SetData(0, bbox);
            DA.SetData(1, new Vector3d(nx, ny, nz));

        }
    }

}

