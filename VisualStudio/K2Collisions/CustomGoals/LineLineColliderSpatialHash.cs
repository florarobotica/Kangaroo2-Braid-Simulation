using System;
using System.Collections.Generic;
using KangarooSolver;
using Rhino.Geometry;
using K2Collisions.DataStructures;

namespace K2Collisions.CustomGoals
{
    public class LineLineColliderSpatialHash : GoalObject
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
        public int[] divA;
        public int[] divB;
        public int[] divC;
        public int[] divD;

        //Store found indices for move and weight vectors
        List<int[]> foundIndices = new List<int[]>();
        List<Line> displayLines = new List<Line>();
        Dictionary<int,int> pIndexDictionary= new Dictionary<int,int>();
        private int n;

        public LineLineColliderSpatialHash(int[] nakedVertices, int[] edgesFlattened, int[][] edges, SpatialGrid3d<int> grid, int[] meshIndex, double r, double k, bool _show)
        {
            n = nakedVertices.Length;

            //Pindices 0 5 6 7 8 11
            //Indexing 0 1 2 3 4  5
            PIndex = nakedVertices;//naked vertices
            Move = new Vector3d[n]; //Initialize all move vector to 0,0,0 for no movement
            Weighting = new double[n]; //Initialize all weights to 0

            Strength = k;
            R1 = R2 = r;
            show = _show;

            //SpatialGrid
            _positions = new Vec3d[n]; //Number of middle points is equal to naked vertice check geometrically if you do not believe it
            _grid = grid; //SpatialGrid
            _meshIndex = meshIndex;

            //Extra
            _edgesFlattened = edgesFlattened;
            _edges = edges;
            for(int i = 0; i < nakedVertices.Length; i++)
                pIndexDictionary.Add(nakedVertices[i],i);
            divA = divB = divC = divD = new int[n];
        }

        public override void Calculate(List<KangarooSolver.Particle> p)
        {

            //All moves and weight each step are initialized to zero
            Move = new Vector3d[n];
            Weighting = new double[n];

            // SpatialGrid
            foundIndices.Clear();
            displayLines.Clear();

            //Get edges middle point
            for (int i = 0; i < _positions.Length; i++)
                _positions[i] = ((p[_edgesFlattened[i * 2]].Position + p[_edgesFlattened[i * 2 + 1]].Position) * 0.5).ToVec3d();

            //Insert particles into the grid
            for (var i = 0; i < _positions.Length; i++) _grid.Insert(_positions[i], i);

            //Search
            double offset = 1.5;
            Vec3d size = new Vec3d(0.1 * 2.0, 0.1 * 2.0, 0.1 * 2.0);

            for (int i = 0; i < _positions.Length; i++)
            {
                Domain3d bbox = new Domain3d(_positions[i] - size * offset, _positions[i] + size * offset); //Search radius

                _grid.Search(bbox, foundIds =>
                {
                    foreach (int j in foundIds)
                    {
                        //Check if if it belongs to the same stripe
                        if (_meshIndex[j] == _meshIndex[i] || j < i) continue; //Check if it does not belong to the stripe
                        foundIndices.Add(new [] { i * 2 + 0, i * 2 + 1, j * 2 + 0, j * 2 + 1 });           
                    }
                });
            }

            //Clear grid
            _grid.Clear();


            //Loop through all found indices
            for (int i = 0; i < foundIndices.Count; i++)
            {
                //Indices used for geometrical caulculation
                //Edges flattened contains naked vertices indices
                int _a = _edgesFlattened[foundIndices[i][0]];
                int _b = _edgesFlattened[foundIndices[i][1]];
                int _c = _edgesFlattened[foundIndices[i][2]];
                int _d = _edgesFlattened[foundIndices[i][3]];

                divA[pIndexDictionary[_a]] += 1;
                divA[pIndexDictionary[_b]] += 1;
                divA[pIndexDictionary[_c]] += 1;
                divA[pIndexDictionary[_d]] += 1;


                L1 = new Line(p[_a].Position, p[_b].Position);
                L2 = new Line(p[_c].Position, p[_d].Position);

                displayLines.Add( new Line((p[_a].Position+p[_b].Position)/2, (p[_c].Position+p[_d].Position)/2));


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
                #endregion

                // get the difference of the two closest points
                Vector3d dP = w + (sc * u) - (tc * v); // =  S1(sc) - S2(tc)
                var Separation = -dP;
                var Overlap = R1 + R2 - Separation.Length; // Move particles only when lines are within the radius

                //Do not forget to sum all weights and move and divide by number of them.

                if (Overlap > 0) //Only activates the goal when particles are within defined radius
                {
                    Separation.Unitize();
                    var Push = Separation * Overlap;

                    Move[pIndexDictionary[_a]] += ((1 - sc) * -Push);
                    Move[pIndexDictionary[_a]] /= 1; //Summing later try with division
                    Move[pIndexDictionary[_b]] += ((sc) * -Push); // / divA[pIndexDictionary[_b]]; //Summing later try with division
                    Move[pIndexDictionary[_b]] /= 1;

                    Move[pIndexDictionary[_c]] += ((1 - tc) * Push);// / divA[pIndexDictionary[_c]]; //Summing later try with division
                    Move[pIndexDictionary[_c]] /= 1;
                    Move[pIndexDictionary[_d]] += ((tc) * Push);// / divA[pIndexDictionary[_d]]; //Summing later try with division
                    Move[pIndexDictionary[_d]] /= 1;

                    Weighting[pIndexDictionary[_a]] = Weighting[pIndexDictionary[_b]] = Weighting[pIndexDictionary[_c]] = Weighting[pIndexDictionary[_d]] = Strength;// / pIndexDictionary[_a];
                }
                else
                {
                    Move[pIndexDictionary[_a]] = Move[pIndexDictionary[_b]] = Move[pIndexDictionary[_c]] = Move[pIndexDictionary[_d]] = Vector3d.Zero;
                    Weighting[pIndexDictionary[_a]] = Weighting[pIndexDictionary[_b]] = Weighting[pIndexDictionary[_c]] = Weighting[pIndexDictionary[_d]] = Strength;
                }


            }

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
