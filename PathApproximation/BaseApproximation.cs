using System;
using System.Collections.Generic;
using System.Linq;
using Library.Generic;
using Plane = Library.Generic.Plane;

namespace Library.PathApproximation {
    public interface IEquation {
        double GetValue(double t);
        bool IsValid(double t);
    }

    public abstract class BaseEquation : IEquation {
        protected readonly double tFrom;
        protected readonly double tTo;

        protected BaseEquation(double tFrom, double tTo) {
            this.tFrom = tFrom;
            this.tTo = tTo;
        }

        public abstract double GetValue(double t);

        public bool IsValid(double t) {
            return tFrom <= t && t <= tTo;
        }
    }

    public abstract class BaseApproximation {
        private bool useAvgNormals;
        private bool reNormalizeNormals;

        protected BaseApproximation(bool useAvgNormals, bool reNormalizeNormals) {
            this.useAvgNormals = useAvgNormals;
            this.reNormalizeNormals = reNormalizeNormals;
        }

        protected struct tx
        {
            public readonly double t;
            public readonly double x;

            public tx(double at, double ax) {
                t = at;
                x = ax;
            }
        }

        protected abstract int GetTXLength();

        protected virtual bool MustUseMaxAvailableTxLength() => false;

        protected abstract IEquation GetEquation(List<tx> txs);

        private int GetNextPointIndex(List<Point> points, int i0, float step) {
            var i1 = i0;
            while (i1 < points.Count && (points[i1] - points[i0]).Magnitude < step) {
                ++i1;
            }

            if (i1 == points.Count) {
                --i1;
            }

            return i1;
        }

        private List<int> GetPointIndexesInRadius(List<Point> points, int i0, int i1, int i2, float r) {
            var results = new List<int>();
            for (var i = i0; i <= i2; ++i) {
                if ((points[i1] - points[i]).Magnitude <= r) {
                    results.Add(i);
                }
            }

            return results;
        }

        private List<Position> ApproximateLine(List<Position> positions, float step, List<Triangle> triangles, bool hasStartFinishPoints) {
            var surfacePoints = new List<Point>();
            var originPoints = new List<Point>();
            var normals = new List<Point>();
            foreach (var position in positions) {
                surfacePoints.Add(position.surfacePoint);
                originPoints.Add(position.originPoint);

                if (useAvgNormals) {
                    var sumNormals = new Point(0, 0, 0);
                    var sumSquare = 0.0f;
                    foreach (var t in trianglesByPoint[position.surfacePoint]) {
                        if (MMath.Dot(position.paintDirection, t.GetNormal()) < 0) {
                            var square = (float)t.GetSquare();
                            sumNormals += t.GetNormal() * square;
                            sumSquare += square;
                        }
                    }

                    if (sumSquare > 0) {
                        normals.Add(sumNormals / sumSquare);
                    }
                    else {
                        normals.Add(-position.paintDirection);
                    }
                }
                else {
                    normals.Add(-position.paintDirection);
                }
            }

            var pointIndexes = new List<int> {0};
            {
                var i0 = 0;
                while (i0 < surfacePoints.Count - 1) {
                    i0 = GetNextPointIndex(surfacePoints, i0, step);
                    if (i0 != surfacePoints.Count - 1) {
                        pointIndexes.Add(i0);
                    }
                }

                pointIndexes.Add(surfacePoints.Count - 1);
            }

            var avgSurfacePoints = new List<Point>{surfacePoints[pointIndexes.First()]};
            var avgOriginPoints = new List<Point>{originPoints[pointIndexes.First()]};
            var avgNormals = new List<Point> {normals[pointIndexes.First()]};
            for (var i = 1; i < pointIndexes.Count - 1; ++i) {
                var indexes = GetPointIndexesInRadius(surfacePoints, pointIndexes[i - 1], pointIndexes[i], pointIndexes[i + 1], step / 2);
                var count = 0;
                var avgSurfacePoint = new Point(0, 0, 0);
                var avgOriginPoint = new Point(0, 0, 0);
                var avgNormal = new Point(0, 0, 0);
                foreach (var j in indexes) {
                    avgSurfacePoint += surfacePoints[j];
                    avgOriginPoint += originPoints[j];
                    avgNormal += normals[j];
                    ++count;
                }

                avgSurfacePoints.Add(avgSurfacePoint / count);
                avgOriginPoints.Add(avgOriginPoint / count);
                avgNormals.Add(avgNormal / count);
            }

            avgSurfacePoints.Add(surfacePoints[pointIndexes.Last()]);
            avgOriginPoints.Add(originPoints[pointIndexes.Last()]);
            avgNormals.Add(normals[pointIndexes.Last()]);

            var xSurfaceEquations = new List<IEquation>();
            var ySurfaceEquations = new List<IEquation>();
            var zSurfaceEquations = new List<IEquation>();
            var xOriginEquations = new List<IEquation>();
            var yOriginEquations = new List<IEquation>();
            var zOriginEquations = new List<IEquation>();
            var xNormalEquations = new List<IEquation>();
            var yNormalEquations = new List<IEquation>();
            var zNormalEquations = new List<IEquation>();
            var maxT = 0.0f;
            {
                var txLength = GetTXLength();

                if (txLength > avgSurfacePoints.Count) {
                    if (MustUseMaxAvailableTxLength()) {
                        txLength = avgSurfacePoints.Count;
                    }
                    else {
                        return positions;
                    }
                }

                var ts = new List<float>{0.0f};
                for (var i = 0; i < txLength - 1; ++i) {
                    ts.Add(ts[i] + (float)MMath.GetDistance(avgSurfacePoints[i + 1], avgSurfacePoints[i]));
                }

                for (var i = 0; i < avgSurfacePoints.Count - txLength + 1; ++i) {
                    if (i > 0) {
                        for (var j = 0; j < txLength - 1; ++j) {
                            ts[j] = ts[j + 1];
                        }

                        ts[txLength - 1] += (float)MMath.GetDistance(avgSurfacePoints[i + txLength - 1], avgSurfacePoints[i + txLength - 2]);
                    }

                    var xSurfaceTXs = new List<tx>();
                    var ySurfaceTXs = new List<tx>();
                    var zSurfaceTxs = new List<tx>();
                    var xOriginTXs = new List<tx>();
                    var yOriginTXs = new List<tx>();
                    var zOriginTXs = new List<tx>();
                    var xNormalTXs = new List<tx>();
                    var yNormalTXs = new List<tx>();
                    var zNormalTXs = new List<tx>();
                    for (var j = 0; j < txLength; ++j) {
                        xSurfaceTXs.Add(new tx(ts[j], avgSurfacePoints[i + j].x));
                        ySurfaceTXs.Add(new tx(ts[j], avgSurfacePoints[i + j].y));
                        zSurfaceTxs.Add(new tx(ts[j], avgSurfacePoints[i + j].z));
                        xOriginTXs.Add(new tx(ts[j], avgOriginPoints[i + j].x));
                        yOriginTXs.Add(new tx(ts[j], avgOriginPoints[i + j].y));
                        zOriginTXs.Add(new tx(ts[j], avgOriginPoints[i + j].z));
                        xNormalTXs.Add(new tx(ts[j], avgNormals[i + j].x));
                        yNormalTXs.Add(new tx(ts[j], avgNormals[i + j].y));
                        zNormalTXs.Add(new tx(ts[j], avgNormals[i + j].z));
                    }

                    xSurfaceEquations.Add(GetEquation(xSurfaceTXs));
                    ySurfaceEquations.Add(GetEquation(ySurfaceTXs));
                    zSurfaceEquations.Add(GetEquation(zSurfaceTxs));
                    xOriginEquations.Add(GetEquation(xOriginTXs));
                    yOriginEquations.Add(GetEquation(yOriginTXs));
                    zOriginEquations.Add(GetEquation(zOriginTXs));
                    xNormalEquations.Add(GetEquation(xNormalTXs));
                    yNormalEquations.Add(GetEquation(yNormalTXs));
                    zNormalEquations.Add(GetEquation(zNormalTXs));
                }

                maxT = ts.Last();
            }

            var newSurfacePoints = new List<Point>();
            var newOriginPoints = new List<Point>();
            var newNormals = new List<Point>();
            {
                var hValues = new List<float>();
                for (var h = step; h < maxT; h += step) {
                    hValues.Add(h);
                }
                for (var h = maxT - step; h > 0; h -= step) {
                    hValues.Add(h);
                }
                hValues.Sort((a, b) => a < b ? -1 : 1);

                var collapsedHValues = new List<List<float>>();
                collapsedHValues.Add(new List<float>{0.0f});
                var currentList = new List<float>();
                foreach (var h in hValues) {
                    if (h < step / 2.0f || maxT - h < step / 2.0f) {
                        continue;
                    }
                    if (currentList.Count > 0 && h - currentList.First() > step / 2.0f) {
                        collapsedHValues.Add(currentList.ToList());
                        currentList.Clear();
                    }
                    currentList.Add(h);
                }
                collapsedHValues.Add(currentList.ToList());
                collapsedHValues.Add(new List<float>{maxT});

                foreach (var hList in collapsedHValues) {
                    var count = 0;
                    var surfacePoint = new Point(0, 0, 0);
                    var originPoint = new Point(0, 0, 0);
                    var normal = new Point(0, 0, 0);
                    foreach (var h in hList) {
                        for (var i = 0; i < xSurfaceEquations.Count; ++i) {
                            if (xSurfaceEquations[i].IsValid(h)) {
                                var sx = xSurfaceEquations[i].GetValue(h);
                                var sy = ySurfaceEquations[i].GetValue(h);
                                var sz = zSurfaceEquations[i].GetValue(h);
                                var ox = xOriginEquations[i].GetValue(h);
                                var oy = yOriginEquations[i].GetValue(h);
                                var oz = zOriginEquations[i].GetValue(h);
                                var nx = xNormalEquations[i].GetValue(h);
                                var ny = yNormalEquations[i].GetValue(h);
                                var nz = zNormalEquations[i].GetValue(h);

                                if (!(sx is double.NaN) && !(sy is double.NaN) && !(sz is double.NaN)) {
                                    ++count;
                                    surfacePoint += new Point((float)sx, (float)sy, (float)sz);
                                    originPoint += new Point((float)ox, (float)oy, (float)oz);
                                    normal += new Point((float)nx, (float)ny, (float)nz);
                                }
                            }
                        }
                    }

                    if (count > 0) {
                        newSurfacePoints.Add(surfacePoint / count);
                        newOriginPoints.Add(originPoint / count);
                        newNormals.Add(normal / count);
                    }
                }
            }

            if (newSurfacePoints.Count == 0) {
                newSurfacePoints = surfacePoints;
                newOriginPoints = originPoints;
                newNormals = normals;
            }

            var upNormal = Point.Up;
            var paintHeight = (float)(positions[0].originPoint - positions[0].surfacePoint).Magnitude;
            var result = new List<Position>();
            for (var i = 0; i < newSurfacePoints.Count; ++i) {
                var positionType = Position.PositionType.Middle;
                if (hasStartFinishPoints && i == 0) {
                    positionType = Position.PositionType.Start;
                }

                if (hasStartFinishPoints && i == newSurfacePoints.Count - 1) {
                    positionType = Position.PositionType.Finish;
                }

                Point n1 = null;
                if (i > 0) {
                    var p1 = newSurfacePoints[i - 1];
                    var p2 = newSurfacePoints[i];
                    n1 = new Point(0, p1.z - p2.z, p2.y - p1.y).Normalized;
                    if (MMath.Dot(upNormal, n1) < 0) {
                        n1 = new Point(-n1.x, -n1.y, -n1.z).Normalized;
                    }

                    n1 *= (float)MMath.GetDistance(p1, p2);
                }

                Point n2 = null;
                if (i + 1 < newSurfacePoints.Count) {
                    var p1 = newSurfacePoints[i];
                    var p2 = newSurfacePoints[i + 1];
                    n2 = new Point(0, p1.z - p2.z, p2.y - p1.y).Normalized;
                    if (MMath.Dot(upNormal, n2) < 0) {
                        n2 = new Point(-n2.x, -n2.y, -n2.z).Normalized;
                    }

                    n2 *= (float)MMath.GetDistance(p1, p2);
                }

                // 2d normals;
                Point n0 = null;
                if (!(n1 is null) && !(n2 is null)) {
                    n0 = (n1 + n2).Normalized;
                }
                else if (!(n1 is null) ) {
                    n0 = normals.Last();
                    // n0 = n1.Normalized;
                }
                else if (!(n2 is null) ) {
                    n0 = normals.First();
                    // n0 = n2.Normalized;
                }
                else {
                    throw new Exception("Magic");
                }

                if (!reNormalizeNormals) {
                    n0 = (newOriginPoints[i] - newSurfacePoints[i]).Normalized;
                }

                result.Add(new Position(newSurfacePoints[i] + paintHeight * n0, -n0, newSurfacePoints[i], positionType));
            }

            var planes = new List<Plane>();
            foreach (var t in triangles) {
                planes.Add(t.GetPlane());
            }

            // Normal denormalization and redirection;
            for (var j = 0; reNormalizeNormals && j < result.Count; ++j) {
                var item = result[j];

                var scales = new List<Tuple<double, Point>>();
                for (var k = 0; k < planes.Count; ++k) {
                    var point = MMath.Intersect(
                        planes[k],
                        new Segment(
                            item.surfacePoint - 1 * item.paintDirection * paintHeight,
                            item.surfacePoint + 1 * item.paintDirection * paintHeight
                        )
                    );
                    if (!(point is null)) {
                        var t = triangles[k];

                        var s0 = t.GetSquare();
                        var s1 = new Triangle(point, t.p1, t.p2).GetSquare();
                        var s2 = new Triangle(point, t.p1, t.p3).GetSquare();
                        var s3 = new Triangle(point, t.p2, t.p3).GetSquare();

                        var dSurface = MMath.GetDistance(item.surfacePoint, point);
                        var normal = item.paintDirection;
                        if (Math.Abs(s0 - s1 - s2 - s3) < 50) {
                            var dOrigin = MMath.GetDistance(item.surfacePoint - normal * paintHeight, point);
                            var length = MMath.GetDistance(item.surfacePoint - normal * paintHeight, item.surfacePoint);

                            if (normal.y < -0.98) {
                                UnityEngine.Debug.Log("qwe");
                            }

                            var scale = double.NaN;
                            var variant = 0;
                            if (Math.Abs(dSurface + dOrigin - length) < 1) {
                                scale = (length + dSurface) / length;
                                variant = 1;
                            }
                            else if (Math.Abs(length + dSurface - dOrigin) < 1) {
                                scale = (length - dSurface) / length;
                                UnityEngine.Debug.Log($"Variant2: {scale} {length} {dSurface} {dOrigin}");
                                variant = 2;
                            }

                            if (!double.IsNaN(scale)) {
                                UnityEngine.Debug.Log($"NewScale: {variant} {normal} {scale} {scale}");
                                scales.Add(new Tuple<double, Point>(scale, point));
                            }
                        }
                    }
                }

                if (scales.Count > 0) {
                    UnityEngine.Debug.Log("ScaleCount: " + scales.Count.ToString());
                    var bestResult = scales.First();
                    var scale = bestResult.Item1;
                    var newOriginPoint = item.surfacePoint - item.paintDirection * paintHeight * (float)scale;
                    var newSurfacePoint = newOriginPoint + item.paintDirection * paintHeight;
                    result[j] = new Position(
                        newOriginPoint,
                        item.paintDirection,
                        newSurfacePoint,
                        item.type
                    );
                }
            }

            return result;
        }

        private readonly Dictionary<Point, List<Triangle>> trianglesByPoint = new Dictionary<Point, List<Triangle>>();

        private float getDistance(Triangle t, Point p) {
            var a = MMath.GetDistance(t.o, p);
            var b = MMath.GetDistance(t.p1, p);
            var c = MMath.GetDistance(t.p2, p);
            var d = MMath.GetDistance(t.p3, p);
            return (float)Math.Min(Math.Min(a, b), Math.Min(c, d));
        }

        private List<Triangle> GetNearestTriangles(Point point, float radius, Dictionary<Point, List<Triangle>> tPoints,
            List<Triangle> triangles) {
            Triangle nearestT = null;
            foreach (var t in triangles) {
                if (nearestT is null || getDistance(t, point) < getDistance(nearestT, point)) {
                    nearestT = t;
                }
            }

            if (nearestT is null) {
                throw new Exception();
            }

            var result = new List<Triangle>();
            var processedTriangles = new Dictionary<Triangle, bool>();
            var queue = new Queue<Triangle>();
            queue.Enqueue(nearestT);
            processedTriangles.Add(nearestT, true);
            while (queue.Count > 0) {
                var t = queue.Dequeue();
                if (getDistance(t, point) < radius) {
                    result.Add(t);
                    foreach (var p in t.GetPoints()) {
                        foreach (var tp in tPoints[p]) {
                            if (!processedTriangles.ContainsKey(tp)) {
                                queue.Enqueue(tp);
                                processedTriangles.Add(tp, true);
                            }
                        }
                    }
                }
            }

            return result;
        }

        private void PrepareData(List<Position> positions, float radius, List<Triangle> triangles) {
            var pointsDict = new Dictionary<Point, List<Triangle>>();
            foreach (var t in triangles) {
                foreach (var p in t.GetPoints()) {
                    if (!pointsDict.ContainsKey(p)) {
                        pointsDict[p] = new List<Triangle>();
                    }

                    pointsDict[p].Add(t);
                }
            }

            foreach (var position in positions) {
                if (!trianglesByPoint.ContainsKey(position.surfacePoint)) {
                    trianglesByPoint.Add(position.surfacePoint,
                        GetNearestTriangles(position.surfacePoint, radius, pointsDict, triangles));
                }
            }
        }

        public List<Position> Approximate(List<Position> positions, float step, List<Triangle> triangles) {
            if (useAvgNormals) {
                PrepareData(positions, 50, triangles);
            }

            var result = new List<Position>();
            var i0 = 0;
            var i1 = 0;
            while (i1 < positions.Count) {
                while (i1 < positions.Count - 1 && positions[i1].type != Position.PositionType.Finish) {
                    ++i1;
                }

                var hasStartFinishPoints = positions[i0].type == Position.PositionType.Start && positions[i1].type == Position.PositionType.Finish;
                result.AddRange(ApproximateLine(positions.GetRange(i0, i1 - i0 + 1), step, triangles, hasStartFinishPoints));

                ++i1;
                i0 = i1;
            }

            return result;
        }
    }
}
