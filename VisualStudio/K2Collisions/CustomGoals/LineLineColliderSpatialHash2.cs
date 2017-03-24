using System;
using System.Collections.Generic;
using KangarooSolver;
using Rhino.Geometry;
using K2Collisions.DataStructures;

namespace K2Collisions.CustomGoals
{
    public class LineLineColliderSpatialHash2 : GoalObject
    {
        public double Strength;
        public double R1, R2;
        public bool show = false;
        public Line L1;
        public Line L2;

        //Spatial grid
        public int[][] _edges;
        public int[] _edgesFlattened;
        public SpatialGrid3d<int> _grid;
        public int[] _meshIndex;
        public Vec3d[] _positions;

        //Store found indices for geometrical caluclation
        List<int> pa = new List<int>();
        List<int> pb = new List<int>();
        List<int> pc = new List<int>();
        List<int> pd = new List<int>();
        //Store found indices for move and weight vectors
        List<int> paI = new List<int>();
        List<int> pbI = new List<int>();
        List<int> pcI = new List<int>();
        List<int> pdI = new List<int>();
        List<Line> displayLines = new List<Line>();


        public LineLineColliderSpatialHash2(int[] nakedVertices, int[] edgesFlattened, int[][] edges, SpatialGrid3d<int> grid, int[] meshIndex, double r, double k, bool _show)
        {
            _edgesFlattened = edgesFlattened;
            _grid = grid;
            _edges = edges;
            _meshIndex = meshIndex;

            //0 16 0 1 1 2 2 3 3 4 4 PIndex
            //0  1 1 2 3 4 5 6 7 8 9 PIndex Indices
            //0  0 0 0 0 0 0 0 0 0 0 Move and Weight

            //PIndex would are number of all edges 0 1 one edge 4 8 another
            PIndex = edgesFlattened;// all edges .___. end points
            Move = new Vector3d[edgesFlattened.Length]; //Initialize all move vector to 0,0,0 for no movement
            Weighting = new double[edgesFlattened.Length]; //Initialize all weights to 0
            _positions = new Vec3d[edges.Length];


            Strength = k;
            R1 = R2 = r;
            show = _show;
        }

        public override void Calculate(List<KangarooSolver.Particle> p)
        {

            //SpatialGrid
            pa.Clear();
            pb.Clear();
            pc.Clear();
            pd.Clear();
            paI.Clear();
            pbI.Clear();
            pcI.Clear();
            pdI.Clear();
            displayLines.Clear();

            Move = new Vector3d[Move.Length]; //Initialize all move vector to 0,0,0 for no movement
            Weighting = new double[Weighting.Length]; //Initialize all weights to 0

            int[] divisionsA = new int[Move.Length];
            int[] divisionsB = new int[Move.Length];
            int[] divisionsC = new int[Move.Length];
            int[] divisionsD = new int[Move.Length];

            //Get edges middle point
            for (int i = 0; i < PIndex.Length * 0.5; i++)
            {
                _positions[i] = new Vec3d(
                    (p[PIndex[i * 2]].Position.X + p[PIndex[i * 2 + 1]].Position.X) * 0.5,
                    (p[PIndex[i * 2]].Position.Y + p[PIndex[i * 2 + 1]].Position.Y) * 0.5,
                    (p[PIndex[i * 2]].Position.Z + p[PIndex[i * 2 + 1]].Position.Z) * 0.5
                    );
            }

            //for (int i = 0; i < _edgesFlattened.Length; i++)
            //{
            //    Rhino.RhinoApp.WriteLine("PIndex " + i.ToString() + " -> " + PIndex[i].ToString() + " edges " + _edgesFlattened[i]);
            //}


            //Rhino.RhinoApp.WriteLine(_edges.Length.ToString());


            //Insert particles into the grid
            for (int i = 0; i < _positions.Length; i++)
                _grid.Insert(_positions[i], i);

            //Search
            double offset = 1.5;
            Vec3d size = new Vec3d(0.1 * 2.0, 0.1 * 2.0, 0.1 * 2.0);



            for (int i = 0; i < _positions.Length; i++)
            {
                int counter = 0;
                Domain3d bbox = new Domain3d(_positions[i] - size * offset, _positions[i] + size * offset); //Search radius

                _grid.Search(bbox, foundIds =>
                {

                    //Rhino.RhinoApp.WriteLine();

                    foreach (int j in foundIds)
                    {
                        //Avoid too many collisions
                        counter++;
                        if (counter > 10)
                            break;

                        //Check if if it belongs to the same stripe
                        if (_meshIndex[j] == _meshIndex[i]) continue; //Check if it does not belong to the stripe
                        if (j <= i) continue;
                        
                            // Rhino.RhinoApp.Write(i.ToString() + " [" + j.ToString() + "] ");
                            //Add indices fo
                            pa.Add(PIndex[i * 2 + 0]);
                            pb.Add(PIndex[i * 2 + 1]);
                            pc.Add(PIndex[j * 2 + 0]);
                            pd.Add(PIndex[j * 2 + 1]);

                            paI.Add(i * 2 + 0);
                            pbI.Add(i * 2 + 1);
                            pcI.Add(j * 2 + 0);
                            pdI.Add(j * 2 + 1);
                        

                    }



                });
            }


            #region math
            //Loop through all found indices
            for (int i = 0; i < pa.Count; i++)
            {
                divisionsA[PIndex[pa[i]]]++;
                divisionsB[PIndex[pb[i]]]++;
                divisionsC[PIndex[pc[i]]]++;
                divisionsD[PIndex[pd[i]]]++;

                //Indices used for geometrical caulculation
                //But for move vector we need to track original PIndex particles
                //L1 = new Line(p[pa[i]].Position, p[pb[i]].Position);
                //L2 = new Line(p[pc[i]].Position, p[pd[i]].Position);
                L1 = new Line(p[PIndex[paI[i]]].Position, p[PIndex[pbI[i]]].Position);
                L2 = new Line(p[PIndex[pcI[i]]].Position, p[PIndex[pdI[i]]].Position);

                displayLines.Add(L1);
                displayLines.Add(L2);

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
                var Overlap = R1 + R2 - Separation.Length; // Move particles only when lines are within the radius

                //if (i == 0)
                //{
                //    Rhino.RhinoApp.WriteLine(Overlap.ToString());
                //    Rhino.RhinoApp.WriteLine(PIndex.Count().ToString());
                //}

                //Do not forget to sum all weights and move and divide by number of them.

                if (Overlap > 0)
                {
                    Separation.Unitize();
                    var Push = Separation * Overlap;

                    Move[paI[i]] = ((1 - sc) * -Push);// / divisionsA[pa[i]]; //Summing later try with division
                    Move[pbI[i]] = ((sc) * -Push);// / divisionsB[pb[i]]; //Summing later try with division

                    Move[pcI[i]] = ((1 - tc) * Push);// / divisionsC[pc[i]]; //Summing later try with division
                    Move[pdI[i]] = ((tc) * Push);//  / divisionsD[pd[i]]; //Summing later try with division

                    Weighting[paI[i]] = Weighting[pbI[i]] = Weighting[pcI[i]] = Weighting[pdI[i]] = Strength;
                }
                else
                {
                    Move[paI[i]] = Move[pbI[i]] = Move[pcI[i]] = Move[pdI[i]] = Vector3d.Zero;
                    Weighting[paI[i]] = Weighting[pbI[i]] = Weighting[pcI[i]] = Weighting[pdI[i]] = 0;
                }


            }
            #endregion

            //Clear grid
            _grid.Clear();



        }

        public override object Output(List<KangarooSolver.Particle> p)
        {
            if (show)
            {
                return displayLines.ToArray();
                //Line[] l = new Line[pa.Count * 2];


                //for (int i = 0; i < pa.Count; i++)
                //{

                //    l[i * 2 + 0] = new Line(p[pa[i]].Position, p[pb[i]].Position);
                //    l[i * 2 + 1] = new Line(p[pc[i]].Position, p[pd[i]].Position);

                //    //l[i * 2 + 0] = new Line(p[paI[i]].Position, p[pbI[i]].Position);
                //    //l[i * 2 + 1] = new Line(p[pcI[i]].Position, p[pdI[i]].Position);

                //    //Point3d pta = new Point3d(
                //    //    (p[PIndex[pa[i]]].Position.X + p[PIndex[pb[i]]].Position.X) * 0.5,
                //    //    (p[PIndex[pa[i]]].Position.Y + p[PIndex[pb[i]]].Position.Y) * 0.5,
                //    //    (p[PIndex[pa[i]]].Position.Z + p[PIndex[pb[i]]].Position.Z) * 0.5
                //    //    );

                //    //Point3d ptb = new Point3d(
                //    //    (p[PIndex[pc[i]]].Position.X + p[PIndex[pd[i]]].Position.X) * 0.5,
                //    //    (p[PIndex[pc[i]]].Position.Y + p[PIndex[pd[i]]].Position.Y) * 0.5,
                //    //    (p[PIndex[pc[i]]].Position.Z + p[PIndex[pd[i]]].Position.Z) * 0.5
                //    //);

                //    //Point3d pta = new Point3d(
                //    //    (p[pa[i]].Position.X + p[pb[i]].Position.X) * 0.5,
                //    //    (p[pa[i]].Position.Y + p[pb[i]].Position.Y) * 0.5,
                //    //    (p[pa[i]].Position.Z + p[pb[i]].Position.Z) * 0.5
                //    //    );

                //    //Point3d ptb = new Point3d(
                //    //    (p[pc[i]].Position.X + p[pd[i]].Position.X) * 0.5,
                //    //    (p[pc[i]].Position.Y + p[pd[i]].Position.Y) * 0.5,
                //    //    (p[pc[i]].Position.Z + p[pd[i]].Position.Z) * 0.5
                //    //);

                //    //l[i + 0] = new Line(pta, ptb);
                //    //l[i + 1] = new Line(pta, ptb);
                //}
                //return l;
            }
            else return null;
        }

    }
}
