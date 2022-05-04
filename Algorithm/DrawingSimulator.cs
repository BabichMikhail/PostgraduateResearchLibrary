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
        public Dictionary<Triangle, Dictionary<Point, double>> paintAmount;
        public List<Triangle> triangles;
        public Dictionary<RobotPathItem, Dictionary<Triangle, Dictionary<Point, double>>> detailedPaintAmountForItems;
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
            if (triangles.Count == 0) {
                return triangles;
            }

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

        public TexturePaintResult ProcessPoints(
            List<RobotPathProcessor> robotPathProcessors,
            List<Triangle> triangles,
            Dictionary<Triangle, List<Point>> points,
            float maxR,
            float paintRadius,
            float paintHeight,
            float paintConsumptionRateGameSizeUnitsCubicMeterPerSecond
        ) {
            var xTriangles = triangles.OrderBy(XFunc).ToList();
            var yTriangles = triangles.OrderBy(YFunc).ToList();
            var zTriangles = triangles.OrderBy(ZFunc).ToList();

            var sumTime = 0.0f;
            var timePositions = new Dictionary<Position, float>();
            var detailedTimePositions = new Dictionary<Position, Dictionary<RobotPathItem, float>>();
            var itemPositionCount = new Dictionary<Position, int>();
            foreach (var rpp in robotPathProcessors) {
                foreach (var item in rpp.GetRobotPathItems()) {
                    DictUtils.FillValueIfNotExists(timePositions, item.a, 0.0f);
                    DictUtils.FillValueIfNotExists(timePositions, item.b, 0.0f);
                    DictUtils.FillValueIfNotExists(detailedTimePositions, item.a, new Dictionary<RobotPathItem, float>());
                    DictUtils.FillValueIfNotExists(detailedTimePositions, item.b, new Dictionary<RobotPathItem, float>());
                    DictUtils.FillValueIfNotExists(detailedTimePositions[item.a], item, 0.0f);
                    DictUtils.FillValueIfNotExists(detailedTimePositions[item.b], item, 0.0f);
                    DictUtils.FillValueIfNotExists(itemPositionCount, item.a, 0);
                    DictUtils.FillValueIfNotExists(itemPositionCount, item.b, 0);

                    var speed = item.GetSpeed(rpp.GetSurfaceSpeed());
                    var distance = (float)MMath.GetDistance(item.a.originPoint, item.b.originPoint);
                    var time = distance / speed / 2.0f;
                    sumTime += time * 2;
                    if (float.IsInfinity(time)) {
                        throw new Exception("Illegal time");
                    }

                    timePositions[item.a] += time;
                    timePositions[item.b] += time;
                    detailedTimePositions[item.a][item] += time;
                    detailedTimePositions[item.b][item] += time;
                    ++itemPositionCount[item.a];
                    ++itemPositionCount[item.b];
                }
            }

            var sum = 0.0;
            var paintAmount = new Dictionary<Triangle, Dictionary<Point, double>>();
            var detailedPaintAmount = new Dictionary<Position, Dictionary<Triangle, Dictionary<Point, double>>>();
            foreach (var timePosition in timePositions) {
                var position = timePosition.Key;
                var time = timePosition.Value;

                if (!detailedPaintAmount.ContainsKey(position)) {
                    detailedPaintAmount.Add(position, new Dictionary<Triangle, Dictionary<Point, double>>());
                }

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

                    DictUtils.FillValueIfNotExists(paintAmount, t, new Dictionary<Point, double>());
                    DictUtils.FillValueIfNotExists(detailedPaintAmount[position], t, new Dictionary<Point, double>());

                    if (points.ContainsKey(t)) {
                        foreach (var p in points[t]) {
                            DictUtils.FillValueIfNotExists(paintAmount[t], p, 0.0);
                            DictUtils.FillValueIfNotExists(detailedPaintAmount[position][t], p, 0.0);
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
                        foreach (var p in points[t]) {
                            var direction = (p - position.originPoint).Normalized;
                            var cos = MMath.Dot(-t.GetNormal().Normalized, direction);

                            var densityTest = 1.0f;
                            {
                                var directionX = new Point(direction.x, direction.y, 0.0f).Normalized;
                                var dotX = Math.Min(1.0f, MMath.Dot(position.paintDirection.Normalized, directionX.Normalized));
                                var tgX = MMath.Sqrt(1 - dotX * dotX) / dotX;
                                var densityX = MMath.GetNormalDistributionProbabilityDensity(paintHeight * tgX, 0.0f, paintRadius / 3.0f);
                                densityTest *= densityX;
                            }
                            {
                                var directionZ = new Point(0.0f, direction.y, direction.z);
                                var dotZ = Math.Min(1.0f, MMath.Dot(position.paintDirection.Normalized, directionZ.Normalized));
                                var tgZ = MMath.Sqrt(1 - dotZ * dotZ) / dotZ;
                                var densityZ = MMath.GetNormalDistributionProbabilityDensity(paintHeight * tgZ, 0.0f, paintRadius / 3.0f);
                                densityTest *= densityZ;
                            }

                            var dot = Math.Min(1.0f, MMath.Dot(position.paintDirection, direction));
                            if (dot > 0) {
                                var tg2 = MMath.Sqrt(1 - dot * dot) / dot;
                                var density = MMath.GetNormalDistributionProbabilityDensity(paintHeight * tg2, 0.0f, paintRadius / 3.0f);
                                density *= MMath.GetNormalDistributionProbabilityDensity(0.0f, 0.0f, paintRadius / 3.0f);

                                var r = (float)MMath.GetDistance(p, position.originPoint);
                                var k = paintConsumptionRateGameSizeUnitsCubicMeterPerSecond * time * density * MMath.Pow(paintHeight / (dot * r), 2) * cos;
                                // var k = paintConsumptionRateGameSizeUnitsCubicMeterPerSecond * time * density * cos * MMath.Pow(paintHeight / r, 2);
                                if (float.IsInfinity(k) || float.IsNaN(k)) {
                                    throw new Exception("Illegal paint amount");
                                }

                                paintAmount[t][p] += k;
                                detailedPaintAmount[position][t][p] += k;
                                sum += k;
                            }
                        }
                    }
                }
            }

            var detailedPaintAmountForItems = new Dictionary<RobotPathItem, Dictionary<Triangle, Dictionary<Point, double>>>();
            foreach (var detailedPaintAmountByPosition in detailedPaintAmount) {
                var position = detailedPaintAmountByPosition.Key;
                var pathItemTimes = detailedTimePositions[position];
                UnityEngine.Debug.Assert(pathItemTimes.Count == 1 || pathItemTimes.Count == 2);
                foreach (var detailedPaintAmountByTriangle in detailedPaintAmountByPosition.Value) {
                    var triangle = detailedPaintAmountByTriangle.Key;
                    foreach (var pointPaintAmount in detailedPaintAmountByTriangle.Value) {
                        var point = pointPaintAmount.Key;
                        foreach (var pathItemTime in pathItemTimes) {
                            var item = pathItemTime.Key;
                            UnityEngine.Debug.Assert(item.a == position || item.b == position);

                            DictUtils.FillValueIfNotExists(detailedPaintAmountForItems, item, new Dictionary<Triangle, Dictionary<Point, double>>());
                            DictUtils.FillValueIfNotExists(detailedPaintAmountForItems[item], triangle, new Dictionary<Point, double>());
                            DictUtils.FillValueIfNotExists(detailedPaintAmountForItems[item][triangle], point, 0.0);
                            detailedPaintAmountForItems[item][triangle][point] += pointPaintAmount.Value / 2.0;
                        }
                    }
                }
            }

            detailedPaintAmount.Clear();
            return new TexturePaintResult {
                paintAmount = paintAmount,
                triangles = triangles,
                detailedPaintAmountForItems = detailedPaintAmountForItems,
            };
        }

        public TexturePaintResult ProcessPointsWithTriangles(
            List<RobotPathProcessor> robotPathProcessors,
            List<Triangle> triangles,
            Dictionary<Triangle, List<Point>> points,
            float maxR,
            float paintRadius,
            float paintHeight,
            float maxTriangleSquare,
            float paintConsumptionRateGameSizeUnitsCubicMeterPerSecond
        ) {
            var maxSquare = maxTriangleSquare;
            var maxSideLength = (float)Math.Sqrt(maxSquare);
            var triangleReplacements = SplitTriangles(triangles, maxSquare, maxSideLength);

            var customTriangles = new List<Triangle>();
            foreach (var t in triangles) {
                if (triangleReplacements.ContainsKey(t)) {
                    if (points.ContainsKey(t)) {
                        foreach (var point in points[t]) {
                            var dSquare = double.MaxValue;
                            var bestTriangle = triangleReplacements[t].First();
                            foreach (var tr in triangleReplacements[t]) {
                                var s0 = tr.GetSquare();
                                var s1 = new Triangle(tr.p1, tr.p2, point).GetSquare();
                                var s2 = new Triangle(tr.p1, tr.p3, point).GetSquare();
                                var s3 = new Triangle(tr.p2, tr.p3, point).GetSquare();
                                var ds = s0 - s1 - s2 - s3;
                                if (ds < dSquare) {
                                    dSquare = ds;
                                    bestTriangle = tr;
                                }
                            }

                            DictUtils.FillValueIfNotExists(points, bestTriangle, new List<Point>());
                            points[bestTriangle].Add(point);
                        }
                    }

                    points.Remove(t);
                    customTriangles.AddRange(triangleReplacements[t]);
                }
                else {
                    customTriangles.Add(t);
                }
            }

            foreach (var t in customTriangles) {
                DictUtils.FillValueIfNotExists(points, t, new List<Point>());
                points[t].Add(t.p1);
                points[t].Add(t.p2);
                points[t].Add(t.p3);
            }

            var result = ProcessPoints(
                robotPathProcessors,
                customTriangles,
                points,
                maxR,
                paintRadius,
                paintHeight,
                paintConsumptionRateGameSizeUnitsCubicMeterPerSecond
            );
            return result;
        }

        public TexturePaintResult ProcessPath(
            List<RobotPathProcessor> robotPathProcessors,
            List<Triangle> triangles,
            float maxR,
            float paintRadius,
            float paintHeight,
            float maxTriangleSquare,
            float paintConsumptionRateGameSizeUnitsCubicMeterPerSecond
        ) {
            return ProcessPointsWithTriangles(
                robotPathProcessors,
                triangles,
                new Dictionary<Triangle, List<Point>>(),
                maxR,
                paintRadius,
                paintHeight,
                maxTriangleSquare,
                paintConsumptionRateGameSizeUnitsCubicMeterPerSecond
            );
        }
    }
}
