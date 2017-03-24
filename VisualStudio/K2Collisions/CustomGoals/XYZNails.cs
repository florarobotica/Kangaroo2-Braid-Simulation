using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using KangarooSolver;
using Rhino.Geometry;
using Particle = KangarooSolver.Particle;

namespace K2Collisions.CustomGoals
{
    public class XYZNails : GoalObject
    {
        public double Strength;

        public Point3d PtA;

        public bool xFix;

        public bool yFix;

        public bool zFix;

        public XYZNails(Point3d P, bool X, bool Y, bool Z, double k)
        {
            base.PPos = new Point3d[] { P };
            base.Move = new Vector3d[1];
            base.Weighting = new double[] { k };
            this.Strength = k;
            this.PtA = P;
            this.xFix = X;
            this.yFix = Y;
            this.zFix = Z;
        }

        public XYZNails(int id, Point3d P, bool X, bool Y, bool Z, double k)
        {
            base.PIndex = new int[] { id };
            base.PPos = new Point3d[] { P };
            base.Move = new Vector3d[1];
            base.Weighting = new double[] { k };
            this.Strength = k;
            this.PtA = P;
            this.xFix = X;
            this.yFix = Y;
            this.zFix = Z;
        }

        public override void Calculate(System.Collections.Generic.List<Particle> p)
        {
            Vector3d vector3d = this.PtA - p[base.PIndex[0]].Position;
            if (!this.xFix)
            {
                vector3d.X = (0.0);
            }
            if (!this.yFix)
            {
                vector3d.Y = (0.0);
            }
            if (!this.zFix)
            {
                vector3d.Z = (0.0);
            }
            base.Move[0] = vector3d;
            base.Weighting[0] = this.Strength;
        }
    }
}
