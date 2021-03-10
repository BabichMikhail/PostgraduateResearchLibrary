using System;

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

    public class SphereTriangle {
        public readonly SpherePoint p1;
        public readonly SpherePoint p2;
        public readonly SpherePoint p3;

        private Triangle originTriangle;

        public SphereTriangle(SphereCoordinatesTransformer transformer, Triangle t) {
            p1 = transformer.Transform(t.p1);
            p2 = transformer.Transform(t.p2);
            p3 = transformer.Transform(t.p3);
        }
    }

    public class SphereCoordinatesTransformer {
        private readonly Point origin;

        public SphereCoordinatesTransformer(Point aOrigin) {
            origin = aOrigin;
        }

        public SpherePoint Transform(Point p) {
            p -= origin;
            var r = p.Magnitude;
            return new SpherePoint(r, (float)Math.Acos(p.z / r), (float)Math.Atan(p.y / p.z));
        }

        public Point InverseTransform(SpherePoint p) {
            return new Point(
                p.r * (float)Math.Sin(p.theta) * (float)Math.Cos(p.phi),
                p.r * (float)Math.Sin(p.theta) * (float)Math.Sin(p.phi),
                p.r * (float)Math.Cos(p.theta)
            ) + origin;
        }
    }
}
