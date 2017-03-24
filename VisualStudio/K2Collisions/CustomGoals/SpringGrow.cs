using System.Collections.Generic;
using KangarooSolver;
using Rhino.Geometry;
using Particle = KangarooSolver.Particle;

namespace K2Collisions.CustomGoals
{
    public class SpringGrow : GoalObject
    {
        public double RestLength;
        public double Stiffness;
        public double max;

      



        public SpringGrow(int s, int e, double l, double max, double k)
        {
            this.PIndex = new int[] { s, e };
            this.Move = new Vector3d[2];
            this.Weighting = new double[2];
            this.RestLength = l;
            this.Stiffness = k;
            this.max = max;
        }

        public SpringGrow(Point3d s, Point3d e, double l, double max, double k)
        {
            this.PPos = new Point3d[] { s, e };
            this.Move = new Vector3d[2];
            this.Weighting = new double[2];
            this.RestLength = l;
            this.Stiffness = k;
            this.max = max;
        }

        public override void Calculate(List<Particle> p)
        {
            if (this.RestLength < this.max - 0.03)
                this.RestLength += 0.01;
            else if (this.RestLength > this.max + 0.03)
                this.RestLength -= 0.01;

            Vector3d position = p[this.PIndex[1]].Position - p[this.PIndex[0]].Position;
            double restLength = 1 - this.RestLength / position.Length;
            Vector3d vector3d = (0.5 * position) * restLength;
            this.Move[0] = vector3d;
            this.Move[1] = -vector3d;
            this.Weighting[0] = 2 * this.Stiffness;
            this.Weighting[1] = 2 * this.Stiffness;
        }



        public override  object Output(List<Particle> p)
        {
            // return new Line(p[this.PIndex[0]].Position, p[this.PIndex[1]].Position);
            return null;
        }
    }
}