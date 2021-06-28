using System;

namespace Library.Generic
{
    public class Point : IEquatable<Point>, IFormattable {
        public readonly float x;
        public readonly float y;
        public readonly float z;

        public Point(float ax, float ay, float az) {
            x = ax;
            y = ay;
            z = az;
        }

        public double Magnitude => Math.Sqrt(SqrMagnitude);
        public double SqrMagnitude => x * x + y * y + z * z;
        public Point Normalized => new Point(x, y, z) / (float)Magnitude;
        public static Point Zero => new Point(0, 0, 0);
        public static Point Up => new Point(0, 1, 0);
        public static Point Down => new Point(0, -1, 0);
        public static Point Left => new Point(-1, 0, 0);
        public static Point Right => new Point(1, 0, 0);
        public static Point Forward => new Point(0, 0, 1);
        public static Point Back => new Point(0, 0, -1);

        public static Point operator +(Point a, Point b) => new Point(a.x + b.x, a.y + b.y, a.z + b.z);
        public static Point operator -(Point a, Point b) => new Point(a.x - b.x, a.y - b.y, a.z - b.z);
        public static Point operator /(Point a, float b) => new Point(a.x / b, a.y / b, a.z / b);
        public static Point operator *(Point a, float b) => new Point(a.x * b, a.y * b, a.z * b);
        public static Point operator *(float a, Point b) => b * a;
        public static Point operator -(Point a) => new Point(-a.x, -a.y, -a.z);
        public static bool operator ==(Point a, Point b) {
            if (a is null && b is null) {
                return true;
            }

            if (a is null || b is null) {
                return false;
            }

            var num1 = a.x - b.x;
            var num2 = a.y - b.y;
            var num3 = a.z - b.z;
            return num1 * num1 + num2 * num2 + num3 * num3 < 9.99999943962493E-11;
        }

        public static bool operator !=(Point a, Point b) => !(a == b);
        public bool Equals(Point other) => this == other;
        public override int GetHashCode() => x.GetHashCode() ^ y.GetHashCode() << 2 ^ z.GetHashCode() >> 2;
        public override string ToString() => $"({x} {y} {z})";
        public string ToString(string format, IFormatProvider formatProvider) => ToString();
    }
}
