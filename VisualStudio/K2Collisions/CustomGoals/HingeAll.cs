using System.Collections.Generic;
using KangarooSolver;
using Rhino.Geometry;
using Particle = KangarooSolver.Particle;

namespace K2Collisions.CustomGoals
{
    public class HingeAll : GoalObject
    {
        public double Strength;

        public double RestAngle;
        public double end;


        private double start = 0.0;
        private double step = 0;

        public HingeAll(int[] P0, int[] P1, int[] P2, int[] P3, double RA, double k)
        {
            int length = P0.Length;
            this.PIndex = new int[length * 4];
            P0.CopyTo(this.PIndex, 0);
            P1.CopyTo(this.PIndex, length);
            P2.CopyTo(this.PIndex, length * 2);
            P3.CopyTo(this.PIndex, length * 3);
            this.Move = new Vector3d[length * 4];
            this.Weighting = new double[length * 4];

            this.Strength = k;
            this.end = k;
            //this.Strength = start;
            this.RestAngle = RA;

            this.step = k*0.001;
        

           

        }

        public override void Calculate(List<Particle> p)
        {
            if (start < end)
            {
                start += step;
                this.Strength = start;
            }
            else
                this.Strength = end;



            int count = this.PIndex.Length / 4;
            for (int i = 0; i < count; i++)
            {
                Point3d position = p[this.PIndex[i]].Position;
                Point3d point3d = p[this.PIndex[count + i]].Position;
                Point3d position2 = p[this.PIndex[count * 2 + i]].Position;
                Point3d point3d2 = p[this.PIndex[count * 3 + i]].Position;
                Vector3d vector3d = point3d - position;
                Vector3d vector3d2 = position2 - position;
                Vector3d vector3d3 = point3d2 - position;
                Vector3d vector3d4 = point3d - position2;
                Vector3d vector3d5 = point3d - point3d2;
                double length = 1.0 / vector3d.Length;
                double num = (vector3d2 - vector3d2 * length * length * vector3d * vector3d).Length;
                double length2 = 0.5 / (num + (vector3d3 - vector3d3 * length * length * vector3d * vector3d).Length);
                double num2 = vector3d2 * vector3d;
                double num3 = vector3d3 * vector3d;
                double num4 = vector3d4 * vector3d;
                double num5 = vector3d5 * vector3d;
                Vector3d vector3d6 = Vector3d.CrossProduct(vector3d2, vector3d);
                Vector3d vector3d7 = Vector3d.CrossProduct(vector3d, vector3d3);
                double num6 = Vector3d.VectorAngle(vector3d6, vector3d7, new Plane(position, vector3d));
                bool flag = num6 > 3.14159265358979;
                if (flag)
                {
                    num6 -= 6.28318530717959;
                }
                double restAngle = num6 - this.RestAngle;
                double length3 = 1.0 / vector3d6.Length;
                double num7 = num2 * length3;
                double num8 = num4 * length3;
                length3 = 1.0 / vector3d7.Length;
                double num9 = num3 * length3;
                double num10 = num5 * length3;
                double num11 = restAngle * length2 * 0.5;
                Vector3d vector3d8 = vector3d6 * num11;
                this.Move[i] = num8 * vector3d8;
                this.Move[count + i] = num7 * vector3d8;
                this.Move[count * 2 + i] = -(num8 + num7) * vector3d8;
                vector3d8 = vector3d7 * num11;
                this.Move[i] += num10 * vector3d8;
                this.Move[count + i] += num9 * vector3d8;
                this.Move[count * 3 + i] = -(num10 + num9) * vector3d8;
                this.Weighting[i] = (this.Weighting[count + i] = (this.Weighting[count * 2 + i] = (this.Weighting[count * 3 + i] = this.Strength)));
            }
        }


        public override  object Output(List<Particle> p)
        {
            return null;
        }
    }
}
