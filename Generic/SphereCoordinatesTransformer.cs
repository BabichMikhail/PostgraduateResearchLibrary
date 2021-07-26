using System;
using System.Collections.Generic;
using System.Linq;

namespace Library.Generic
{
    public class SpherePoint {
        public readonly float r;
        public readonly float theta;
        public readonly float phi;

        public SpherePoint(float aR, float aTheta, float aPhi) {
            r = aR;
            theta = aTheta;
            phi = aPhi;
        }
    }

    public class SphereEdge {
        public readonly SpherePoint p1;
        public readonly SpherePoint p2;

        public SphereEdge(SpherePoint aP1, SpherePoint aP2) {
            p1 = aP1;
            p2 = aP2;
        }
    }

    public class SphereTriangle {
        public readonly SpherePoint p1;
        public readonly SpherePoint p2;
        public readonly SpherePoint p3;

        public readonly float minR;
        public readonly float maxR;
        public readonly float minTheta;
        public readonly float maxTheta;
        // minPhi in radians from 0 to 4 Pi. minPhi <= maxPhi
        public readonly float minPhi;
        // maxPhi in radians from 0 to 4 Pi. maxPhi >= minPHi
        public readonly float maxPhi;

        private readonly Point originPoint;
        private readonly Triangle originTriangle;

        public SphereTriangle(SpherePoint aP1, SpherePoint aP2, SpherePoint aP3, Point aOriginPoint, Triangle aOriginTriangle) {
            p1 = aP1;
            p2 = aP2;
            p3 = aP3;

            originPoint = aOriginPoint;
            originTriangle = aOriginTriangle;

            minR = Math.Min(p1.r, Math.Min(p2.r, p3.r));
            maxR = Math.Max(p1.r, Math.Max(p2.r, p3.r));
            minTheta = Math.Min(p1.theta, Math.Min(p2.theta, p3.theta));
            maxTheta = Math.Max(p1.theta, Math.Max(p2.theta, p3.theta));

            var points = new List<SpherePoint> {p1, p2, p3};
            points = points.OrderBy(x => x.phi).ToList();

            var phi1 = points[0].phi;
            var phi2 = points[1].phi;
            var phi3 = points[2].phi;

            if (phi2 - phi1 > Math.PI) {
                var phi = phi1;
                phi1 = phi2;
                phi2 = phi3;
                phi3 = phi + 2 * (float)Math.PI;
            }

            if (phi3 - phi2 > Math.PI) {
                var phi = phi2;
                phi1 = phi3;
                phi2 = phi1 + 2 * (float)Math.PI;
                phi3 = phi + 2 * (float)Math.PI;
            }

            minPhi = phi1;
            maxPhi = phi3;

            if (phi2 - phi1 < Math.PI && phi3 - phi2 < Math.PI && phi1 + Math.PI < phi3) {
                minPhi = -(float)Math.PI;
                maxPhi = (float)Math.PI;
            }
        }

        private bool IsPointInTriangle(Triangle t, Point p) {
            var t1 = new Triangle(t.p1, t.p2, p);
            var t2 = new Triangle(t.p1, t.p3, p);
            var t3 = new Triangle(t.p2, t.p3, p);
            var s1 = t1.GetSquare();
            var s2 = t2.GetSquare();
            var s3 = t3.GetSquare();
            var s = t.GetSquare();
            var sumS = s1 + s2 + s3;
            var k = (t1.GetSquare() + t2.GetSquare() + t3.GetSquare()) / t.GetSquare();
            return (t1.GetSquare() + t2.GetSquare() + t3.GetSquare()) / t.GetSquare() < 1f + 1e-6f;
        }

        public bool HasOverlap(SphereTriangle other) {
            return false;
            var ok = other.minTheta < maxTheta && other.maxTheta > minTheta && other.minPhi < maxPhi && other.maxPhi > minPhi;
            if (true || ok) {
                // var p = new Plane(originTriangle.p1, originTriangle.p2, originTriangle.p3);

                var plane1 = originTriangle.GetPlane();
                var plane2 = other.originTriangle.GetPlane();
                var o1 = originTriangle.GetCenterOfTheInscribedCircle();
                var o2 = other.originTriangle.GetCenterOfTheInscribedCircle();

                if (MMath.Intersect(plane1, new Segment(originPoint, o1)) is null) {
                    UnityEngine.Debug.Assert(false);
                }
                if (MMath.Intersect(plane2, new Segment(originPoint, o2)) is null) {
                    UnityEngine.Debug.Assert(false);
                }

                while (true) {
                    var target = (o1 + o2) / 2.0f;
                    var direction = target - originPoint;
                    var l0 = new Segment(originPoint, originPoint + direction.Normalized * (float)direction.Magnitude * 6f);

                    var _p1 = MMath.Intersect(plane1, l0);
                    var _p2 = MMath.Intersect(plane2, l0);

                    var ok1 = !(_p1 is null) && IsPointInTriangle(originTriangle, _p1);
                    var ok2 = !(_p2 is null) && IsPointInTriangle(other.originTriangle, _p2);

                    if (ok1 && ok2) {
                        IsPointInTriangle(originTriangle, _p1);
                        IsPointInTriangle(other.originTriangle, _p2);
                        return true;
                    }
                    else if (ok1 && o1 != target) {
                        o1 = target;
                    }
                    else if (ok2 && o2 != target) {
                        o2 = target;
                    }
                    else {
                        IsPointInTriangle(originTriangle, _p1);
                        IsPointInTriangle(other.originTriangle, _p2);
                        return false;
                    }
                }

                {
                    // var l1 = new Segment(originPoint, other.originTriangle.p1);
                    // var lp1 = MMath.Intersect(p, l1, true);
                    // if (!(lp1 is null) && IsPointInTriangle(originTriangle, lp1)) {
                    //     return true;
                    // }
                    //
                    // var l2 = new Segment(originPoint, other.originTriangle.p2);
                    // var lp2 = MMath.Intersect(p, l2, true);
                    // if (!(lp2 is null) && IsPointInTriangle(originTriangle, lp2)) {
                    //     return true;
                    // }
                    //
                    // var l3 = new Segment(originPoint, other.originTriangle.p3);
                    // var lp3 = MMath.Intersect(p, l3, true);
                    // if (!(lp3 is null) && IsPointInTriangle(originTriangle, lp3)) {
                    //     return true;
                    // }
                    //
                    // var lo = new Segment(originPoint, other.originTriangle.o);
                    // var lpo = MMath.Intersect(p, lo, true);
                    // if (!(lpo is null) && IsPointInTriangle(originTriangle, lpo)) {
                    //     return true;
                    // }
                }
            }

            return false;
        }

        public List<SphereEdge> GetEdges() {
            return new List<SphereEdge> {
                new SphereEdge(p1, p2),
                new SphereEdge(p1, p3),
                new SphereEdge(p2, p3),
            };
        }
    }

    public class SphereCoordinatesTransformer {
        private readonly Point origin;

        public SphereCoordinatesTransformer(Point aOrigin) {
            origin = aOrigin;
        }

        public SpherePoint Transform(Point p) {
            p -= origin;
            var r = (float)p.Magnitude;
            return new SpherePoint(r, (float)Math.Acos(p.z / r), (float)Math.Atan2(p.y, p.x));
        }

        public Point InverseTransform(SpherePoint p) {
            return new Point(
                p.r * (float)Math.Sin(p.theta) * (float)Math.Cos(p.phi),
                p.r * (float)Math.Sin(p.theta) * (float)Math.Sin(p.phi),
                p.r * (float)Math.Cos(p.theta)
            ) + origin;
        }

        public SphereTriangle Transform(Triangle t) {
            return new SphereTriangle(Transform(t.p1), Transform(t.p2), Transform(t.p3), origin, t);
        }

        public Triangle InverseTransform(SphereTriangle t) {
            return new Triangle(InverseTransform(t.p1), InverseTransform(t.p2), InverseTransform(t.p3));
        }
    }
}
