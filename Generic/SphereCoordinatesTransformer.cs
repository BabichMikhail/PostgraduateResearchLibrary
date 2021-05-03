using System;
using System.Collections.Generic;

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

        private Triangle originTriangle;

        public SphereTriangle(SpherePoint aP1, SpherePoint aP2, SpherePoint aP3) {
            p1 = aP1;
            p2 = aP2;
            p3 = aP3;
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
            return new SpherePoint(r, (float)Math.Acos(p.z / r), (float)Math.Atan(p.y / p.z));
        }

        public Point InverseTransform(SpherePoint p) {
            return new Point(
                p.r * (float)Math.Sin(p.theta) * (float)Math.Cos(p.phi),
                p.r * (float)Math.Sin(p.theta) * (float)Math.Sin(p.phi),
                p.r * (float)Math.Cos(p.theta)
            ) + origin;
        }

        public SphereTriangle Transform(Triangle t) {
            return new SphereTriangle(Transform(t.p1), Transform(t.p2), Transform(t.p3));
        }

        public Triangle InverseTransform(SphereTriangle t) {
            return new Triangle(InverseTransform(t.p1), InverseTransform(t.p2), InverseTransform(t.p3));
        }
    }
}
