using System;
using System.Collections.Generic;
using System.Linq;
using Library.Generic;
using Library.RobotPathBuilder;
using Plane = Library.Generic.Plane;

namespace Library.Algorithm
{
    public struct Color {
        public float r;
        public float g;
        public float b;
        public float a;

        public Color(float r, float g, float b, float a = 1.0f) {
            this.r = r;
            this.g = g;
            this.b = b;
            this.a = a;
        }

        public static Color operator +(Color a, Color b) => new Color(a.r + b.r, a.g + b.g, a.b + b.b, a.a + b.a);
        public static Color operator -(Color a, Color b) => new Color(a.r - b.r, a.g - b.g, a.b - b.b, a.a - b.a);
    }

    public class TexturePaintResult {
        public Dictionary<Triangle, Dictionary<Point, float>> paintAmount;
        public List<Triangle> triangles;
    }

    public class DrawingSimulator {
        private int LowerIndex<T>(List<T> triangles, Func<T, float> f, float value) {
            var a = -1;
            var b = triangles.Count - 1;

            while (b - a > 1) {
                var m = (a + b) / 2;
                var v = f(triangles[m]);
                if (v < value) {
                    a = m;
                }
                else {
                    b = m;
                }
            }

            return b;
        }

        private List<T> GetTrianglesInRadius<T>(List<T> triangles, Func<T, float> f, float min, float max) {
            var a = LowerIndex(triangles, f, min);
            var b = LowerIndex(triangles, f, max);
            return triangles.GetRange(a, b - a + 1);
        }

        private float XFunc(Triangle t) => Math.Min(Math.Min(t.p1.x, t.p2.x), t.p3.x);
        private float YFunc(Triangle t) => Math.Min(Math.Min(t.p1.y, t.p2.y), t.p3.y);
        private float ZFunc(Triangle t) => Math.Min(Math.Min(t.p1.z, t.p2.z), t.p3.z);
        private float RMinFunc(SphereTriangle t) => t.minR;
        private float RMaxFunc(SphereTriangle t) => t.maxR;
        private float PhiMinFunc(SphereTriangle t) => t.minPhi;
        private float PhiMaxFunc(SphereTriangle t) => t.maxPhi;
        private float ThetaMinFunc(SphereTriangle t) => t.minTheta;
        private float ThetaMaxFunc(SphereTriangle t) => t.maxTheta;

        private Triangle BuildTriangleWithLikeNormal(Triangle t, Point p1, Point p2, Point p3) {
            var newTriangle = new Triangle(p1, p2, p3);
            if (MMath.Dot(newTriangle.GetNormal(), t.GetNormal()) < 0) {
                newTriangle = new Triangle(p1, p3, p2);
            }

            return newTriangle;
        }

        private List<Triangle> SplitTriangleByLongSide(Triangle t, int cuts) {
            var points = t.GetPoints();

            var edges = t.GetEdges();
            var longestEdge = edges.First();
            foreach (var e in edges) {
                if (e.Length() > longestEdge.Length()) {
                    longestEdge = e;
                }
            }

            var basePoint = points.FirstOrDefault(p => p != longestEdge.p1 && p != longestEdge.p2);

            var result = new List<Triangle>();
            var from = longestEdge.p1;
            var to = longestEdge.p2;

            var lastPoint = from;
            for (var i = 0; i < cuts - 1; ++i) {
                var newPoint = (from * (cuts - 1 - i) + to * (i + 1)) / cuts;
                result.Add(BuildTriangleWithLikeNormal(t, basePoint, lastPoint, newPoint));
                lastPoint = newPoint;
            }
            result.Add(BuildTriangleWithLikeNormal(t, basePoint, lastPoint, to));

            return result;
        }

        private List<Triangle> TrySplitTriangle(Triangle triangle, float maxSquare, float maxSideLength) {
            var result = new List<Triangle>();

            var q = new Queue<Triangle>();
            q.Enqueue(triangle);

            while (q.Count > 0) {
                var t = q.Dequeue();

                var sideLength = 0.0;
                t.GetEdges().ForEach(x => sideLength = Math.Max(sideLength, x.Length()));

                var s = t.GetSquare();
                if (s > maxSquare || sideLength > maxSideLength) {
                    SplitTriangleByLongSide(t, 2).ForEach(x => q.Enqueue(x));
                }
                else if (triangle != t) {
                    result.Add(t);
                }
            }

            return result;
        }

        private Dictionary<Triangle, List<Triangle>> SplitTriangles(List<Triangle> triangles, float maxSquare, float maxSideLength) {
            var result = new Dictionary<Triangle, List<Triangle>>();
            foreach (var t in triangles) {
                var replaces = TrySplitTriangle(t, maxSquare, maxSideLength);
                if (replaces.Count > 0) {
                    result.Add(t, replaces);
                }
            }

            return result;
        }

        public TexturePaintResult ProcessPath(
            List<RobotPathProcessor> robotPathProcessors, List<Triangle> triangles, float maxR, float paintRadius, float paintHeight, float maxTriangleSquare,
            float paintConsumptionRateGameSizeUnitsCubicMeterPerSecond
        ) {
            var maxSquare = maxTriangleSquare;
            var maxSideLength = (float)Math.Sqrt(maxSquare);
            var triangleReplacements = SplitTriangles(triangles, maxSquare, maxSideLength);

            var customTriangles = new List<Triangle>();
            foreach (var t in triangles) {
                if (triangleReplacements.ContainsKey(t)) {
                    customTriangles.AddRange(triangleReplacements[t]);
                }
                else {
                    customTriangles.Add(t);
                }
            }

            var xTriangles = customTriangles.OrderBy(XFunc).ToList();
            var yTriangles = customTriangles.OrderBy(YFunc).ToList();
            var zTriangles = customTriangles.OrderBy(ZFunc).ToList();

            var timePositions = new Dictionary<Position, float>();
            foreach (var rpp in robotPathProcessors) {
                foreach (var item in rpp.GetRobotPathItems()) {
                    if (!timePositions.ContainsKey(item.a)) {
                        timePositions.Add(item.a, 0.0f);
                    }

                    if (!timePositions.ContainsKey(item.b)) {
                        timePositions.Add(item.b, 0.0f);
                    }

                    var speed = item.GetSpeed(rpp.GetSurfaceSpeed(), rpp.GetMaxOriginSpeed());
                    var distance = (float)MMath.GetDistance(item.a.surfacePoint, item.b.surfacePoint);
                    var time = distance / speed;

                    timePositions[item.a] += time / 2.0f;
                    timePositions[item.b] += time / 2.0f;
                }
            }

            var paintAmount = new Dictionary<Triangle, Dictionary<Point, float>>();
            foreach (var timePosition in timePositions) {
                var position = timePosition.Key;
                var time = timePosition.Value;

                var trianglesSet = new Dictionary<Triangle, int>();
                {
                    var xRange = GetTrianglesInRadius(xTriangles, XFunc, position.originPoint.x - maxR, position.originPoint.x + maxR);
                    var yRange = GetTrianglesInRadius(yTriangles, YFunc, position.originPoint.y - maxR, position.originPoint.y + maxR);
                    var zRange = GetTrianglesInRadius(zTriangles, ZFunc, position.originPoint.z - maxR, position.originPoint.z + maxR);
                    foreach (var selectedTriangles in new List<List<Triangle>> {xRange, yRange, zRange}) {
                        foreach (var t in selectedTriangles) {
                            if (trianglesSet.ContainsKey(t)) {
                                ++trianglesSet[t];
                            }
                            else {
                                trianglesSet.Add(t, 1);
                            }
                        }
                    }
                }

                var plane = new Plane(position.paintDirection, position.originPoint);
                var n1 = (position.originPoint - plane.GetSomePoint()).Normalized;
                var n2 = -position.paintDirection.Normalized;
                var n3 = new Point(n1.y * n2.z - n1.z * n2.y, n1.z * n2.x - n1.x * n2.z, n1.x * n2.y - n1.y * n2.x).Normalized;
                var sphereTransformer = new SphereCoordinatesTransformer(position.originPoint);

                var trianglesForPosition = new List<Triangle>();
                foreach (var kv in trianglesSet) {
                    var t = kv.Key;

                    if (!paintAmount.ContainsKey(t)) {
                        paintAmount.Add(t, new Dictionary<Point, float>());
                    }

                    foreach (var p in t.GetPoints()) {
                        if (!paintAmount[t].ContainsKey(p)) {
                            paintAmount[t].Add(p, 0.0f);
                        }
                    }

                    if (kv.Value == 3) {
                        var r = (float)MMath.GetDistance(t, position.originPoint);
                        if (r < maxR) {
                            var direction = (t.o - position.originPoint).Normalized;
                            var cos = MMath.Dot(-t.GetNormal().Normalized, direction);
                            if (cos > 0) {
                                trianglesForPosition.Add(t);
                            }
                        }
                    }
                }

                var sphereTriangles = new List<SphereTriangle>();
                var triangleBySphereTriangle = new Dictionary<SphereTriangle, Triangle>();
                foreach (var t in trianglesForPosition) {
                    var st = sphereTransformer.Transform(t);
                    sphereTriangles.Add(st);
                    triangleBySphereTriangle.Add(st, t);
                }

                var blockedSphereTriangles = new Dictionary<SphereTriangle, bool>();

                // TODO fix overlapping
                var rMinRange = sphereTriangles.OrderBy(RMinFunc).ToList();
                var rMaxRange = sphereTriangles.OrderBy(RMaxFunc).ToList();
                var phiMinRange = sphereTriangles.OrderBy(PhiMinFunc).ToList();
                var phiMaxRange = sphereTriangles.OrderBy(PhiMaxFunc).ToList();
                var thetaMinRange = sphereTriangles.OrderBy(ThetaMinFunc).ToList();
                var thetaMaxRange = sphereTriangles.OrderBy(ThetaMaxFunc).ToList();

                foreach (var st in rMinRange) {
                    if (!blockedSphereTriangles.ContainsKey(st)) {
                        var ts1 = GetTrianglesInRadius(thetaMinRange, x => x.minTheta, st.minTheta, st.maxTheta);
                        foreach (var t in ts1) {
                            if (st != t && !blockedSphereTriangles.ContainsKey(t) && MMath.HasOverlap(st, t)) {
                                blockedSphereTriangles.Add(t, true);
                            }
                        }

                        var ts2 = GetTrianglesInRadius(thetaMaxRange, x => x.maxTheta, st.minTheta, st.maxTheta);
                        foreach (var t in ts2) {
                            if (st != t && !blockedSphereTriangles.ContainsKey(t) && MMath.HasOverlap(st, t)) {
                                blockedSphereTriangles.Add(t, true);
                            }
                        }
                    }
                }

                foreach (var st in sphereTriangles) {
                    if (!blockedSphereTriangles.ContainsKey(st)) {
                        var t = triangleBySphereTriangle[st];

                        var direction = (t.o - position.originPoint).Normalized;
                        var cos = MMath.Dot(-t.GetNormal().Normalized, direction);

                        var dot = MMath.Dot(position.paintDirection, direction);
                        dot = MMath.Round(dot * 1e4f) / 1e4f;
                        var x = MMath.Sqrt(1 - dot * dot) / dot;
                        var density = MMath.GetNormalDistributionProbabilityDensity(paintHeight * x, 0.0f, paintRadius / 3.0f);

                        foreach (var p in t.GetPoints()) {
                            var r = (float)MMath.GetDistance(p, position.originPoint);
                            var k = density * cos * MMath.Pow(paintHeight / r, 2) * paintConsumptionRateGameSizeUnitsCubicMeterPerSecond * time;
                            paintAmount[t][p] += k;
                        }
                    }
                }
            }

            return new TexturePaintResult {
                paintAmount = paintAmount,
                triangles = customTriangles,
            };
        }
    }
}
