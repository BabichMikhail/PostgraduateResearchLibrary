using System.Collections.Generic;

namespace Library.Generic
{
    public class Edge {
        public readonly Point p1;
        public readonly Point p2;

        public Edge(Point aP1, Point aP2) {
            p1 = aP1;
            p2 = aP2;
        }

        public override int GetHashCode() => new KeyValuePair<Point, Point>(p1, p2).GetHashCode();

        public List<Point> GetPoints() => new List<Point> {p1, p2};

        public bool HasPoint(Point p) => p1 == p || p2 == p;

        public float Dx() => p2.x - p1.x;

        public float Dy() => p2.y - p1.y;

        public float Dz() => p2.z - p1.z;

        public float Length() => (p2 - p1).Magnitude;

        public static bool operator ==(Edge a, Edge b) => a.p1 == b.p1 && a.p2 == b.p2 || a.p1 == b.p2 && a.p2 == b.p1;

        public static bool operator !=(Edge a, Edge b) => !(a == b);

        public Plane GetPlane() {
            var v = p2 - p1;
            var d = -(p2.x * v.x + p2.y * v.y + p2.z * v.z);
            var plane = new Plane(v.x, v.y, v.z, d);

            return plane;
        }
    }
}
