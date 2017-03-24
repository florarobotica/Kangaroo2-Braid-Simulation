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
    public class PlasticNails : GoalObject
    {
        public Point3d Location;

        public double Limit;

        public PlasticNails(int Id, Point3d P, double R, double k)
        {
            base.PIndex = new int[] { Id };
            base.Move = new Vector3d[1];
            base.Weighting = new double[] { k };
            this.Location = P;
            this.Limit = R;
        }

        public PlasticNails(Point3d P, double R, double k)
        {
            base.PPos = new Point3d[] { P };
            base.Move = new Vector3d[1];
            base.Weighting = new double[] { k };
            this.Location = P;
            this.Limit = R;
        }

        public override void Calculate(System.Collections.Generic.List<Particle> p)
        {
            Point3d position = p[base.PIndex[0]].Position;
            Vector3d vector3d = this.Location - position;
            base.Move[0] = vector3d;

            double num = vector3d.Length - this.Limit;
            if (num > 0.0)
            {
                vector3d.Unitize();
                vector3d *= num;
                this.Location -= vector3d;
                base.Move[0] -= vector3d;
            }
        }
    }
}
