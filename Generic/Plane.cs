using System;

namespace Library.Generic
{
    public struct Plane {
        public readonly float a;
        public readonly float b;
        public readonly float c;
        public float d;

        private readonly Point p1;
        private readonly Point p2;
        private readonly Point p3;

        public Plane(Point aP1, Point aP2, Point aP3) {
            p1 = aP1;
            p2 = aP2;
            p3 = aP3;

            a = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
            b = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
            c = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
            d = -(p1.x * a + p1.y * b + p1.z * c);
        }

        public Point GetNormal() {
            return new Point(a, b, c).Normalized;
        }

        public Point GetSomePoint() {
            return new Point(0, 0, -d / c);
        }

        public double GetDistance(Point p) {
            return Math.Abs((double)a * (double)p.x + (double)b * (double)p.y + (double)c * (double)p.z + (double)d) /
                Math.Sqrt((double)a * (double)a + (double)b * (double)b + (double)c * (double)c);
        }

        public double GetDenominator() {
            return Math.Sqrt((double)a * (double)a + (double)b * (double)b + (double)c * (double)c);
        }

        public override int GetHashCode() {
            return new Point(a, b, c).GetHashCode() + new Point(a, b, d).GetHashCode();
        }
    }
}
