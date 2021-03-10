using System;
using System.Collections.Generic;

namespace Library.Generic
{
    public class CartesianCoordinatesTransformer {
        private readonly Point origin;
        private readonly Point n1;
        private readonly Point n2;
        private readonly Point n3;

        private readonly Point rn1;
        private readonly Point rn2;
        private readonly Point rn3;

        public CartesianCoordinatesTransformer(Point aOrigin, Point aN1, Point aN2, Point aN3) {
            origin = aOrigin;
            n1 = aN1;
            n2 = aN2;
            n3 = aN3;

            var d = (float)MMath.GetDeterminant(new List<List<double>> {
                new List<double>{n1.x, n2.x, n3.x},
                new List<double>{n1.y, n2.y, n3.y},
                new List<double>{n1.z, n2.z, n3.z},
            });
            rn1 = new Point(n1.x, n2.x, n3.x) / d;
            rn2 = new Point(n1.y, n2.y, n3.y) / d;
            rn3 = new Point(n1.z, n2.z, n3.z) / d;
        }

        public Point Transform(Point point) {
            point -= origin;

            var x = rn1.x * point.x + rn2.x * point.y + rn3.x * point.z;
            var y = rn1.y * point.x + rn2.y * point.y + rn3.y * point.z;
            var z = rn1.z * point.x + rn2.z * point.y + rn3.z * point.z;

            return new Point(x, y, z);
        }

        public Point InverseTransform(Point point) {
            var x = n1.x * point.x + n2.x * point.y + n3.x * point.z;
            var y = n1.y * point.x + n2.y * point.y + n3.y * point.z;
            var z = n1.z * point.x + n2.z * point.y + n3.z * point.z;

            return new Point(x, y, z) + origin;
        }
    }
}
