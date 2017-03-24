using System;
using System.Collections.Generic;
using System.Linq;
using KangarooSolver;
using KangarooSolver.Goals;
using Rhino.Geometry;
using K2Collisions.DataStructures;
using Particle = KangarooSolver.Particle;

namespace K2Collisions.CustomGoals
{
    public class LineLineAll : GoalObject
    {
        public double Strength, R1, R2;
        public bool show = false;
        public Line L1, L2;
        List<Line> lines = new List<Line>();

        //SpatialGrid
        SpatialGrid3d<int> grid;
        Box box;
        Domain3d domain;
        Vector3d subdivision;
        Vec3d[] positions;
        List<int[]> nep = new List<int[]>();

        List<Line> preview = new List<Line>();

        private int counter = 0;

        public LineLineAll(double r, double k, bool _show, Box bbox, Vector3d sub, Domain3d dom, Vec3d[] pos, List<int[]> nep, SpatialGrid3d<int> grid, int counter )
        {
            //Do this in calculation mehtod
            PIndex = new int[counter];
            Move = new Vector3d[counter];
            Weighting = new double[counter];

            Strength = k;
            R1 = R2 = r;
            show = _show;

            this.box = bbox;
            this.subdivision = sub;
            this.domain = dom;
            this.positions = pos;
            this.nep = nep;
            this.grid = grid;

            this.counter = counter;

        }


        public override void Calculate(List<KangarooSolver.Particle> p)
        {
            lines = new List<Line>();

            //Store particles that are influenced by collision
            List<int> aa = new List<int>();
            List<int> bb = new List<int>();
            List<int> cc = new List<int>();
            List<int> dd = new List<int>();

            // Get current edges middle points
            for (int i = 0; i < positions.Length; i++)
                positions[i] = ((p[nep[i][0]].Position + p[nep[i][1]].Position) * 0.5).ToVec3d();

            //Insert particles into grid
            for (int i = 0; i < positions.Length; i++)
                grid.Insert(positions[i], i);

            //Rhino.RhinoApp.WriteLine(grid.ItemCount.ToString());
            //Search
            double offset = 3;
            for (int i = 0; i < positions.Length; i++)
            {
           
                Domain3d bbox = new Domain3d(positions[i] - new Vec3d(0.2, 0.2, 0.2) * offset, positions[i] + new Vec3d(0.2, 0.2, 0.2) * offset); //Search radius
                grid.Search(bbox, foundIds =>
                {
                    foreach (int j in foundIds)
                    {
                        if (j <= i) continue;
                        //mapping lines middle points back to particles
                       // if (_meshIndex[j] == _meshIndex[i] || j < i) continue; //Check if it does not belong to the stripe
                        //if (
                          // ((p[nep[i][0]].Position + p[nep[i][1]].Position)*0.5).DistanceTo(((p[nep[j][0]].Position + p[nep[j][1]].Position)* 0.5)) < R1*1000000)
                        //{
                        
                            aa.Add(nep[i][0]);
                            bb.Add(nep[i][1]);
                            cc.Add(nep[j][0]);
                            dd.Add(nep[j][1]);
                       //}
                    }
                });
            }

            //Override Pindex and move
            PIndex = new int[aa.Count*4];
            aa.CopyTo(this.PIndex, 0);
            bb.CopyTo(this.PIndex, aa.Count);
            cc.CopyTo(this.PIndex, aa.Count * 2);
            dd.CopyTo(this.PIndex, aa.Count * 3);

            Move = new Vector3d[aa.Count * 4];
            Weighting = new double[aa.Count * 4];


            //Initialize PIndex here instead of constructor
            //PIndex = new int[aa.Count * 4];

            //Move = new Vector3d[aa.Count * 4];
            //Weighting = new double[aa.Count * 4];



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


                  // lines.Add(new Line((p[aa[j]].Position + p[bb[j]].Position) * 0.5, ((p[cc[j]].Position + p[dd[j]].Position) * 0.5)));

                    lines.Add(new Line((p[PIndex[count * 0 + j]].Position + p[PIndex[count * 1 + j]].Position) * 0.5, ((p[PIndex[count * 2 + j]].Position + p[PIndex[count * 3 + j]].Position) * 0.5)));

                    // lines.Add(new Line(p[PIndex[count * 0 + j]].Position, p[PIndex[count * 1 + j]].Position));
                    // lines.Add(new Line(p[PIndex[count * 2 + j]].Position, p[PIndex[count * 3 + j]].Position));

                    Weighting[count * 0 + j] = Weighting[count * 1 + j] = Weighting[count * 2 + j] = Weighting[count * 3 + j] = Strength;
                }
                else
                {
                    Move[count * 0 + j] = Move[count * 1 + j] = Move[count * 2 + j] = Move[count * 3 + j] = Vector3d.Zero;
                    Weighting[count * 0 + j] = Weighting[count * 1 + j] = Weighting[count * 2 + j] = Weighting[count * 3 + j] = 0;
                }
            }

            grid.Clear();


        }

        public override object Output(List<Particle> p)
        {
            return lines;
        }
    }
}
