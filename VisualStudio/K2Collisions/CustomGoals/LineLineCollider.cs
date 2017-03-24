using System;
using System.Collections.Generic;
using KangarooSolver;
using Rhino.Geometry;

namespace K2Collisions.CustomGoals
{
    public class LineLineCollider : GoalObject
    {
        public double Strength, R1, R2;
        public bool show = false;
        public Line L1, L2;
        List<Line> lines = new List<Line>();
        public LineLineCollider(int[] a, int[] b, int[] c, int[] d, double r, double k, bool _show)
        {

            lines = new List<Line>(a.Length * 2);

            int length = a.Length;
            

            PIndex = new int[length * 4];
            a.CopyTo(this.PIndex, 0);
            b.CopyTo(this.PIndex, length);
            c.CopyTo(this.PIndex, length * 2);
            d.CopyTo(this.PIndex, length * 3);

            Move = new Vector3d[length * 4];
            Weighting = new double[length * 4];

            Strength = k;
            R1 = R2 = r;
            show = _show;
        }

        public LineLineCollider(Line L1, Line L2, double r1, double r2, double k, bool _show)
        {
            Strength = k;

            PPos = new Point3d[4] { L1.From, L1.To, L2.From, L2.To };
            Move = new Vector3d[4];
            Weighting = new double[4] { k, k, k, k };

            R1 = r1;
            R2 = r2;

            show = _show;
        }

        public override void Calculate(List<KangarooSolver.Particle> p)
        {
            int count = (int)this.PIndex.Length / 4;

            for (int j = 0; j < count; j++)
            {

                L1 = new Line(p[PIndex[count * 0 + j]].Position, p[PIndex[count * 1 + j]].Position);
                L2 = new Line(p[PIndex[count * 2 + j]].Position, p[PIndex[count * 3 + j]].Position);

                #region math

                Vector3d u = L1.To - L1.From;
                Vector3d v = L2.To - L2.From;
                Vector3d w = L1.From - L2.From;
                double a = u * u;
                double b = u * v;
                double c = v * v;
                double d = u * w;
                double e = v * w;
                double D = a * c - b * b;
                double sc, sN, sD = D;
                double tc, tN, tD = D;

                // compute the line parameters of the two closest points
                if (D < 1e-8)
                {
                    // the lines are almost parallel
                    sN = 0.0; // force using point P0 on segment S1
                    sD = 1.0; // to prevent possible division by 0.0 later
                    tN = e;
                    tD = c;
                }
                else
                {
                    sN = b * e - c * d;
                    tN = a * e - b * d;

                    if (sN < 0.0)
                    {
                        // sc < 0 => the s=0 edge is visible
                        sN = 0.0;
                        tN = e;
                        tD = c;
                    }
                    else if (sN > sD)
                    {
                        // sc > 1  => the s=1 edge is visible
                        sN = sD;
                        tN = e + b;
                        tD = c;
                    }
                }

                if (tN < 0.0)
                {
                    // tc < 0 => the t=0 edge is visible
                    tN = 0.0;
                    // recompute sc for this edge
                    if (-d < 0.0)
                        sN = 0.0;
                    else if (-d > a)
                        sN = sD;
                    else
                    {
                        sN = -d;
                        sD = a;
                    }
                }
                else if (tN > tD)
                {
                    // tc > 1  => the t=1 edge is visible
                    tN = tD;
                    // recompute sc for this edge
                    if ((-d + b) < 0.0)
                        sN = 0;
                    else if ((-d + b) > a)
                        sN = sD;
                    else
                    {
                        sN = (-d + b);
                        sD = a;
                    }
                }

                // finally do the division to get sc and tc
                sc = (Math.Abs(sN) < 1e-8 ? 0.0 : sN / sD);
                tc = (Math.Abs(tN) < 1e-8 ? 0.0 : tN / tD);

                // get the difference of the two closest points
                Vector3d dP = w + (sc * u) - (tc * v); // =  S1(sc) - S2(tc)

                #endregion


                var Separation = -dP;
                var Overlap = R1 + R2 - Separation.Length;
                if (Overlap > 0)
                {
                    Separation.Unitize();
                    var Push = Separation * Overlap;

                    Move[count * 0 + j] = (1 - sc) * -Push;
                    Move[count * 1 + j] = (sc) * -Push;

                    Move[count * 2 + j] = (1 - tc) * Push;
                    Move[count * 3 + j] = (tc) * Push;

                   // lines.Add( new Line(p[PIndex[count * 0 + j]].Position, p[PIndex[count * 1 + j]].Position));
                 //   lines.Add(new Line(p[PIndex[count * 2 + j]].Position, p[PIndex[count * 3 + j]].Position));

                    lines.Add(new Line((p[PIndex[count * 0 + j]].Position + p[PIndex[count * 1 + j]].Position) * 0.5, ((p[PIndex[count * 2 + j]].Position + p[PIndex[count * 3 + j]].Position) * 0.5)));


                    Weighting[count * 0 + j] = Weighting[count * 1 + j] = Weighting[count * 2 + j] = Weighting[count * 3 + j] = Strength;
                }
                else
                {
                    Move[count * 0 + j] = Move[count * 1 + j] = Move[count * 2 + j] = Move[count * 3 + j] = Vector3d.Zero;
                    Weighting[count * 0 + j] = Weighting[count * 1 + j] = Weighting[count * 2 + j] = Weighting[count * 3 + j] = 0;
                }
            }
        }

        public override object Output(List<KangarooSolver.Particle> p)
        {
            //if (show)
            //{
            //    return lines.ToArray();
            //    //int count = (int)this.PIndex.Length / 4;
            //    //Line[] l = new Line[count*2];

            //    //for (int j = 0; j < count; j++)
            //    //{
            //    //    l[j * 2] = new Line(p[PIndex[count * 0 + j]].Position, p[PIndex[count * 1 + j]].Position);
            //    //    l[j * 2 + 1] = new Line(p[PIndex[count * 2 + j]].Position, p[PIndex[count * 3 + j]].Position);
            //    //}

            //    //return l;
            //} else
                return lines;
        }

    }
}
