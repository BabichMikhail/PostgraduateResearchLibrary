using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Library.Generic;
using PathFinders;
using Edge = Library.Generic.Edge;
using Plane = Library.Generic.Plane;
using Point = Library.Generic.Point;
using Position = Library.Generic.Position;
using Triangle = Library.Generic.Triangle;

namespace Library.PathFinders
{
    public class IntersectionsWithSurfacesPathFinder : IPathFinder {
        private class TrianglePath {
            public readonly Triangle triangle;
            public readonly Edge edge;

            public TrianglePath(Triangle aTriangle, Edge aEdge) {
                triangle = aTriangle;
                edge = aEdge;
            }
        }

        private readonly float paintRadius;
        private readonly float paintHeight;
        private readonly float paintLateralAllowance;
        private readonly float paintLongitudinalAllowance;

        public IntersectionsWithSurfacesPathFinder(float aPaintRadius, float aPaintHeight, float aPaintLateralAllowance, float aPaintLongitudinalAllowance) {
            paintRadius = aPaintRadius;
            paintHeight = aPaintHeight;
            paintLateralAllowance = aPaintLateralAllowance;
            paintLongitudinalAllowance = aPaintLongitudinalAllowance;
        }

        public List<Position> GetPath(List<Triangle> triangles) {
            var a = new Point(1, 0, 0);
            var b = new Point(1, 1, 0);
            var c = new Point(1, 1, 1);

            var basePlane = new Plane(a, b, c);

            var a1 = new Point(a.x * MMath.Cos(MMath.PI / 2) - a.z * MMath.Sin(MMath.PI / 2), a.y, a.x * MMath.Sin(MMath.PI / 2) + a.z * MMath.Cos(MMath.PI / 2));
            var b1 = new Point(b.x * MMath.Cos(MMath.PI / 2) - b.z * MMath.Sin(MMath.PI / 2), b.y, b.x * MMath.Sin(MMath.PI / 2) + b.z * MMath.Cos(MMath.PI / 2));
            var c1 = new Point(c.x * MMath.Cos(MMath.PI / 2) - c.z * MMath.Sin(MMath.PI / 2), c.y, c.x * MMath.Sin(MMath.PI / 2) + c.z * MMath.Cos(MMath.PI / 2));
            var plane1 = new Plane(a1, b1, c1);
            var plane2 = new Plane(a1, b1, c1);

            var maxPlaneD = 0.0f;
            foreach (var t in triangles) {
                foreach (var p in t.GetPoints()) {
                    maxPlaneD = Math.Max(p.Magnitude, maxPlaneD);
                }
            }

            var basePlaneD = 2 * maxPlaneD * (float)basePlane.GetDenominator();
            var plane1D = 2 * maxPlaneD * (float)plane1.GetDenominator();
            var plane2D = 2 * maxPlaneD * (float)plane2.GetDenominator();

            basePlane.d = basePlaneD;
            plane1.d = plane1D;
            plane2.d = -plane2D;

            foreach (var t in triangles) {
                foreach (var p in t.GetPoints()) {
                    basePlaneD = Math.Min((float)MMath.GetDistance(basePlane, p), basePlaneD);
                    plane1D = Math.Min((float)MMath.GetDistance(plane1, p), plane1D);
                    plane2D = Math.Min((float)MMath.GetDistance(plane2, p), plane2D);
                }
            }

            basePlane.d -= (basePlaneD - 2 * paintLateralAllowance) * (float)basePlane.GetDenominator();
            plane1.d -= (plane1D - 15 - paintLateralAllowance) * (float)plane1.GetDenominator();
            plane2.d += (plane2D - 15 - paintLateralAllowance) * (float)plane2.GetDenominator();

            {
                var maxDistance = MMath.GetDistance(plane2, plane1);//.GetSomePoint()));
                foreach (var t in triangles) {
                    foreach (var p in t.GetPoints()) {
                        Debug.Assert(MMath.GetDistance(plane1, p) < maxDistance);
                        Debug.Assert(MMath.GetDistance(plane2, p) < maxDistance);
                    }
                }
            }

            var fixedPathPartsByDistance = new Dictionary<double, List<TrianglePath>>();
            foreach (var pathPart in SplitObjectByDistance(basePlane, triangles)) {
                var edgesByPoint = new Dictionary<Point, List<Edge>>();
                foreach (var tp in pathPart.Value) {
                    AddEdge(ref edgesByPoint, tp.edge.p1, tp.edge);
                    AddEdge(ref edgesByPoint, tp.edge.p2, tp.edge);
                }

                var singlePoints = new List<Point>();
                var doublePoints = new List<Point>();
                var wrongPoints = new List<Point>();
                foreach (var ep in edgesByPoint) {
                    if (ep.Value.Count == 1) {
                        singlePoints.Add(ep.Key);
                    }
                    else if (ep.Value.Count == 2) {
                        doublePoints.Add(ep.Key);
                    }
                    else {
                        wrongPoints.Add(ep.Key);
                    }
                }

                if (wrongPoints.Count > 0) {
                    Debug.Assert(false);
                }

                var substitutes = new Dictionary<Point, Point>();
                var processedPointIndexes = new Dictionary<int, bool>();
                for (var i = 0; i < singlePoints.Count; ++i) {
                    var minDistance = 0.0;
                    var nearestIdx = -1;
                    for (var j = i + 1; j < singlePoints.Count; ++j) {
                        var d = (singlePoints[i] - singlePoints[j]).Magnitude;
                        if (i != j && !processedPointIndexes.ContainsKey(i) && !processedPointIndexes.ContainsKey(j) && (nearestIdx == -1 || minDistance > d)) {
                            nearestIdx = j;
                            minDistance = d;
                        }
                    }

                    if (nearestIdx != -1 && minDistance < 5.0f) {
                        processedPointIndexes.Add(i, true);
                        processedPointIndexes.Add(nearestIdx, true);

                        var m = (singlePoints[i] + singlePoints[nearestIdx]) / 2;
                        substitutes.Add(singlePoints[i], m);
                        substitutes.Add(singlePoints[nearestIdx], m);

                        Debug.Assert((singlePoints[i] - singlePoints[nearestIdx]).Magnitude < 1e-4);
                    }
                }

                var trianglePaths = new List<TrianglePath>();
                foreach (var tp in pathPart.Value) {
                    var p1 = tp.edge.p1;
                    if (substitutes.ContainsKey(p1)) {
                        p1 = substitutes[p1];
                    }

                    var p2 = tp.edge.p2;
                    if (substitutes.ContainsKey(p2)) {
                        p2 = substitutes[p2];
                    }

                    var edge = new Edge(p1, p2);
                    trianglePaths.Add(new TrianglePath(tp.triangle, edge));
                }

                fixedPathPartsByDistance.Add(pathPart.Key, trianglePaths);

                Debug.Assert(true);
            }

            var pathPartsByDistance = fixedPathPartsByDistance;
            var result = new List<Position>();
            // For debug. We can specify distance from fromPlane to pathParts
            // var lineNumber = 1;
            foreach (var pathPart in pathPartsByDistance) {
                // if (!(pathPart.Key > 1950 - lineNumber * 200 && pathPart.Key < 2150 - lineNumber * 200)) {
                //     continue;
                // }
                var fromPlane = plane1;
                var toPlane = plane2;

                var edges = new List<Edge>();
                var originalNormalByEdge = new Dictionary<Edge, Point>();
                foreach (var tp in pathPart.Value) {
                    edges.Add(tp.edge);
                    originalNormalByEdge.Add(tp.edge, tp.triangle.GetPlane().GetNormal().Normalized);
                }

                var pointsDict = new Dictionary<Point, bool>();
                var edgesByPoint = new Dictionary<Point, List<Edge>>();
                foreach (var edge in edges) {
                    AddPoint(ref pointsDict, edge.p1);
                    AddPoint(ref pointsDict, edge.p2);

                    AddEdge(ref edgesByPoint, edge.p1, edge);
                    AddEdge(ref edgesByPoint, edge.p2, edge);
                }

                var subResults = GetBodyPositions(edges, fromPlane, toPlane, originalNormalByEdge);

                foreach (var subResult in subResults) {
                    {
                        var i0 = 0;
                        var i1 = i0 + 1;
                        while (i1 < subResult.Count - 1 && (subResult[i0].surfacePoint - subResult[i1].surfacePoint).Magnitude < 5) {
                            ++i1;
                        }

                        var pos0 = subResult[i0];
                        var pos1 = subResult[i1];
                        var pd = pos0.paintDirection;
                        var surface0 = pos0.surfacePoint;
                        var surface1 = pos1.surfacePoint;
                        var dir = (surface0 - surface1).Normalized;
                        result.Add(new Position(pos0.originPoint + dir * paintLongitudinalAllowance, pd, pos0.surfacePoint + dir * paintLongitudinalAllowance, Position.PositionType.Start));
                    }

                    result.AddRange(subResult);

                    {
                        var i0 = subResult.Count - 1;
                        var i1 = i0 - 1;
                        while (i1 > 0 && (subResult[i0].surfacePoint - subResult[i1].surfacePoint).Magnitude < 5) {
                            --i1;
                        }

                        var pos0 = subResult[i0];
                        var pos1 = subResult[i1];
                        var pd = pos0.paintDirection;
                        var surface0 = pos0.surfacePoint;
                        var surface1 = pos1.surfacePoint;
                        var dir = (surface0 - surface1).Normalized;
                        result.Add(new Position(pos0.originPoint + dir * paintLongitudinalAllowance, pd, pos0.surfacePoint + dir * paintLongitudinalAllowance, Position.PositionType.Finish));
                    }
                }
            }

            return result;
        }

        Point GetPointUsingBinarySearch(Point a, Point b, Plane basePlane, double distance) {
            var da = MMath.GetDistance(basePlane, a);
            var db = MMath.GetDistance(basePlane, b);
            if (da > db) {
                MUtils.Swap(ref a, ref b);
                MUtils.Swap(ref da, ref db);
            }

            while (Math.Abs(da - distance) > 1e-4) {
                var p = (a + b) / 2;
                var dp = MMath.GetDistance(basePlane, p);

                if (Math.Abs(dp - distance) <= 1e-4 || dp <= distance) {
                    a = p;
                    da = dp;
                }
                else if (dp > distance) {
                    b = p;
                    db = dp;
                }
                else {
                    throw new Exception("Unexpected behavior in binary search.");
                }
            }

            return a;
        }

        List<Point> GetFilteredPoints(List<Point> originalVertices, Triangle triangle, Plane basePlane) {
            var dict = new Dictionary<int, Point>();
            foreach (var v in originalVertices) {
                var code = v.GetHashCode();
                if (!dict.ContainsKey(code)) {
                    dict.Add(code, v);
                }
            }

            var vertices = dict.Values.ToList();
            while (vertices.Count > 2) {
                var m = new List<List<double>>();
                for (var i = 0; i < vertices.Count; ++i) {
                    m.Add(new List<double>());
                    for (var j = 0; j < vertices.Count; ++j) {
                        m[i].Add(0);
                    }
                }

                var i1 = -1;
                var i2 = -1;
                for (var i = 0; i < vertices.Count; ++i) {
                    for (var j = i + 1; j < vertices.Count; ++j) {
                        m[i][j] = m[j][i] = MMath.GetDistance(vertices[i], vertices[j]);
                        if (i1 == -1 || m[i][j] < m[i1][i2]) {
                            i1 = i;
                            i2 = j;
                        }
                    }
                }

                var d1 = MMath.GetDistance(basePlane, vertices[i1]);
                var d2 = MMath.GetDistance(basePlane, vertices[i2]);

                var newVertices = new List<Point>();
                for (var i = 0; i < vertices.Count; ++i) {
                    if (i != i1 && i != i2) {
                        newVertices.Add(vertices[i]);
                    }
                }

                if (d1 < d2) {
                    newVertices.Add(vertices[i1]);
                }
                else {
                    newVertices.Add(vertices[i2]);
                }

                Debug.Assert(newVertices.Count == 2);
                if (newVertices.Count != 2) {
                    Debug.Assert(false);
                }
                vertices = newVertices;
            }

            if (vertices.Count > 2) {
                Debug.Assert(false);
            }

            return vertices;
        }

        Edge GetDirectedEdge(Plane plane, Edge e) {
            return MMath.GetDistance(plane, e.p1) < MMath.GetDistance(plane, e.p2) ? new Edge(e.p1, e.p2) : new Edge(e.p2, e.p1);
        }

        public List<List<Position>> GetBodyPositions(List<Edge> edges, Plane fromPlane, Plane toPlane, Dictionary<Edge, Point> normalByEdge) {
            var distanceBetweenSequences = 2 * paintLongitudinalAllowance;
            var eSequences = new List<List<Edge>>();
            var usedEdges = new Dictionary<Edge, bool>();
            var usedPoints = new Dictionary<Point, bool>();
            var directedEdges = new Dictionary<Edge, Edge>();

            while (true) {
                Edge de0 = null;
                while (true) {
                    Edge le1 = null;
                    Edge le2 = null;
                    if (eSequences.Count > 0) {
                        le2 = eSequences.Last().Last();
                        var dle2 = directedEdges[le2];
                        for (var i = eSequences.Count - 1; le1 is null && i >= 0; --i) {
                            var eSequence = eSequences[i];
                            for (var j = eSequence.Count - 1; le1 is null && j >= 0; --j) {
                                var dle = directedEdges[eSequence[j]];
                                if (MMath.GetDistance(dle2.p2, dle.p1) > 20) {
                                    le1 = eSequence[j];
                                }
                            }
                        }
                    }

                    Edge e0 = null;
                    foreach (var e in edges) {
                        if (!usedEdges.ContainsKey(e)) {
                            var cFromPlane = fromPlane;
                            var minDistance = 0.0;
                            var isNewPath = le2 is null || MMath.GetDistance(e, le2.p2) >= distanceBetweenSequences;

                            if (!(le2 is null)) {
                                if (!isNewPath) {
                                    var dle2 = directedEdges[le2];
                                    var de = dle2;
                                    if (!(le1 is null)) {
                                        var dle1 = directedEdges[le1];
                                        de = new Edge(dle1.p1, dle2.p2);
                                    }

                                    cFromPlane = de.GetPlane();
                                    cFromPlane.d += (float)(cFromPlane.GetDenominator() * MMath.GetDistance(fromPlane, de.p2));
                                    UnityEngine.Debug.Assert(MMath.GetDistance(cFromPlane, de.p1) < MMath.GetDistance(cFromPlane, de.p2));
                                    minDistance = MMath.GetDistance(cFromPlane, de.p2);
                                }

                                if (isNewPath) {
                                    var dle2 = directedEdges[le2];
                                    minDistance = MMath.GetDistance(fromPlane, dle2.p2);
                                }
                            }

                            var e1 = GetDirectedEdge(cFromPlane, e);
                            var e2 = e0 is null ? null : GetDirectedEdge(cFromPlane, e0);

                            var ok = true;

                            // ok = ok && n1.y > -1 / Math.Sqrt(2);
                            // ok = ok && (le2 is null || MMath.Dot(normalByEdge[le2], n1) > 0);

                            if (ok && !isNewPath && !(le2 is null)) {
                                var dle2 = directedEdges[le2];
                                ok = ok && MMath.Dot(e1.p2 - e1.p1, dle2.p2 - dle2.p1) > 0;
                            }

                            if (ok && !isNewPath && !(le2 is null)) {
                                var distance = MMath.GetDistance(le2.p2, e1.p1);
                                for (var i = 0; i < eSequences.Count; ++i) {
                                    for (var j = 0; j < eSequences[i].Count; ++j) {
                                        if (i < eSequences.Count - 1 || j < eSequences[i].Count - 1) {
                                            var de = directedEdges[eSequences[i][j]];
                                            distance = Math.Min(distance, MMath.GetDistance(de.p2, e1.p1));
                                        }
                                    }
                                }

                                ok = ok && Math.Abs(distance - MMath.GetDistance(le2.p2, e1.p1)) < 1e-6;
                                if (!ok) {
                                    usedEdges.Add(e, true);
                                    continue;
                                }
                            }

                            if (ok && usedPoints.ContainsKey(e1.p2)) {
                                usedEdges.Add(e, true);
                                continue;
                            }

                            ok = ok && MMath.GetDistance(cFromPlane, e1) >= minDistance;

                            if (!(e2 is null)) {
                                ok = ok && (
                                    Math.Abs(MMath.GetDistance(cFromPlane, e1) - MMath.GetDistance(cFromPlane, e2)) < 1e-4 ||
                                    MMath.GetDistance(cFromPlane, e1) + 5 < MMath.GetDistance(cFromPlane, e2)
                                );

                                if (ok && Math.Abs(MMath.GetDistance(cFromPlane, e1) - MMath.GetDistance(cFromPlane, e2)) < 5) {
                                    ok = ok && e1.Dx() > 1e-6 && e1.Dz() > 1e-6 &&
                                        e1.Dy() / Math.Sqrt(e1.Dx() * e1.Dx() + e1.Dz() * e1.Dz()) > e2.Dy() / Math.Sqrt(e2.Dx() * e2.Dx() + e2.Dz() * e2.Dz());
                                }
                            }

                            if (ok && !(e2 is null)) {
                                ok = ok && MMath.GetDistance(fromPlane, e1) - 50 < MMath.GetDistance(fromPlane, e2);
                            }

                            if (ok) {
                                if (!(e0 is null)) {
                                    directedEdges.Remove(e0);
                                }
                                e0 = e;
                                directedEdges.Add(e, e1);
                            }
                        }
                    }

                    if (e0 is null) {
                        break;
                    }

                    if (eSequences.Count > 0) {
                        var rle = eSequences.Last().Last();
                        if (rle.HasPoint(e0.p1) || rle.HasPoint(e0.p2)) {
                            eSequences.Last().Add(e0);
                        }
                        else {
                            eSequences.Add(new List<Edge>{e0});
                        }
                    }
                    else {
                        eSequences.Add(new List<Edge>{e0});
                    }

                    usedEdges.Add(e0, true);

                    if (!usedPoints.ContainsKey(e0.p1)) {
                        usedPoints.Add(e0.p1, true);
                    }

                    if (!usedPoints.ContainsKey(e0.p2)) {
                        usedPoints.Add(e0.p2, true);
                    }
                }

                break;
            }

            {
                var newESequences = new List<List<Edge>>();
                for (var i = 0; i < eSequences.Count; ++i) {
                    var eSequence = eSequences[i];
                    if (i == 0 || MMath.GetDistance(eSequences[i - 1].Last(), eSequence.First()) >= distanceBetweenSequences) {
                        newESequences.Add(eSequence);
                    }
                    else {
                        newESequences.Last().AddRange(eSequence);
                    }
                }

                eSequences = newESequences;
            }

            var results = new List<List<Position>>();
            {
                foreach (var eSequence in eSequences) {
                    var result = new List<Position>();
                    foreach (var e in eSequence) {
                        var de = directedEdges[e];
                        var n = normalByEdge[e];
                        result.Add(new Position(de.p1 + n * paintHeight, -n, de.p1, Position.PositionType.Middle));
                        result.Add(new Position(de.p2 + n * paintHeight, -n, de.p2, Position.PositionType.Middle));
                    }

                    results.Add(result);
                }
            }

            return results;
        }

        Dictionary<double, List<TrianglePath>> SplitObjectByDistance(Plane basePlane, List<Triangle> triangles) {
            var minDistanceToFigure = 1e9;
            foreach (var t in triangles) {
                foreach (var p in t.GetPoints()) {
                    minDistanceToFigure = Math.Min(minDistanceToFigure, MMath.GetDistance(basePlane, p));
                }
            }
            var offset = minDistanceToFigure - paintLateralAllowance;

            var lineWidth = 2 * paintRadius;
            var edgeByAddedPoint = new Dictionary<Point, Edge>();
            var pathPartsByDistance = new Dictionary<double, List<TrianglePath>>();
            foreach (var t in triangles) {
                var minDistance = MMath.GetDistance(basePlane, t.p1);
                var maxDistance = MMath.GetDistance(basePlane, t.p1);

                foreach (var v in new List<Point>{t.p2, t.p3}) {
                    var distance = MMath.GetDistance(basePlane, v);
                    minDistance = Math.Min(minDistance, distance);
                    maxDistance = Math.Max(maxDistance, distance);
                }

                var edges = t.GetEdges();
                for (var distance = Math.Ceiling((minDistance - offset) / lineWidth) * lineWidth + offset; distance <= maxDistance; distance += lineWidth) {
                    var minTDistance = MMath.GetDistance(basePlane, edges.First().p1);
                    var maxTDistance = MMath.GetDistance(basePlane, edges.First().p1);

                    foreach (var e in edges) {
                        foreach (var p in e.GetPoints()) {
                            var d = MMath.GetDistance(basePlane, p);
                            maxTDistance = Math.Max(d, maxTDistance);
                            minTDistance = Math.Min(d, minTDistance);
                        }
                    }

                    var vertices = new List<Point>();
                    if (maxTDistance - minTDistance >= 0) {
                        foreach (var e in edges) {
                            var d1 = MMath.GetDistance(basePlane, e.p1);
                            var d2 = MMath.GetDistance(basePlane, e.p2);

                            var minD = Math.Min(d1, d2);
                            var maxD = Math.Max(d1, d2);
                            if (maxD >= minD && minD <= distance && distance <= maxD) {
                                var point = GetPointUsingBinarySearch(e.p1, e.p2, basePlane, distance);
                                if (Math.Abs(MMath.GetDistance(basePlane, point) - distance) > 1e-4) {
                                    Debug.Assert(false);
                                }

                                vertices.Add(point);
                                if (!edgeByAddedPoint.ContainsKey(point)) {
                                    edgeByAddedPoint.Add(point, e);
                                }
                            }
                        }

                        var fPoints = GetFilteredPoints(vertices, t, basePlane);

                        Edge newEdge = null;
                        if (fPoints.Count == 1 && vertices.Count == 2 && fPoints.GetHashCode() == vertices[0].GetHashCode() && vertices[0].GetHashCode() == vertices[1].GetHashCode()) {
                            // Edge with one point [fPoints[0], fPoints[0]]
                        }
                        else if (fPoints.Count == 2) {
                            newEdge = new Edge(fPoints[0], fPoints[1]);
                            if (fPoints[0].z > 1022 && fPoints[0].z < 1023 || fPoints[1].z > 1022 && fPoints[1].z < 1023) {
                                Debug.Assert(false);
                            }
                        }
                        else {
                            Debug.Assert(false);
                        }

                        if (!(newEdge is null)) {
                            var tp = new TrianglePath(t, newEdge);
                            if (!pathPartsByDistance.ContainsKey(distance)) {
                                pathPartsByDistance.Add(distance, new List<TrianglePath>{tp});
                            }
                            else {
                                pathPartsByDistance[distance].Add(tp);
                            }
                        }
                    }
                }
            }

            return pathPartsByDistance;
        }

        void AddPoint(ref Dictionary<Point, bool> d, Point p) {
            if (!d.ContainsKey(p)) {
                d.Add(p, true);
            }
        }

        void AddEdge(ref Dictionary<Point, List<Edge>> d, Point p, Edge e) {
            var p0 = p;
            if (d.ContainsKey(p0)) {
                d[p0].Add(e);
            }
            else {
                d.Add(p0, new List<Edge>{e});
            }
        }
    }
}
