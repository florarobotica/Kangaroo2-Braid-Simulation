using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace K2Collisions.DataStructures
{
    public static class RhinoExtensions
    {

        public static Vec3d ToVec3d(this Point3d point)
        {
            return new Vec3d(point.X, point.Y, point.Z);
        }

        public static Domain3d ToDomain3d(this BoundingBox bbox)
        {
            Vec3d p0 = bbox.Min.ToVec3d();
            Vec3d p1 = bbox.Max.ToVec3d();
            return new Domain3d(p0, p1);
        }

    }
}
