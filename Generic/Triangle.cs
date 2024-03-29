using System;
using System.Collections.Generic;

namespace Library.Generic
{
    public class Triangle {
        public readonly Point p1;
        public readonly Point p2;
        public readonly Point p3;
        public readonly Point o;
        private readonly Point rawN;

        public readonly double l1;
        public readonly double l2;
        public readonly double l3;

        public Triangle(Point aP1, Point aP2, Point aP3) {
            p1 = aP1;
            p2 = aP2;
            p3 = aP3;

            l1 = (p2 - p3).Magnitude;
            l2 = (p1 - p3).Magnitude;
            l3 = (p1 - p2).Magnitude;

            o = new Point(
                (float)((l3 * p3.x + l2 * p2.x + l1 * p1.x) / (l3 + l2 + l1)),
                (float)((l3 * p3.y + l2 * p2.y + l1 * p1.y) / (l3 + l2 + l1)),
                (float)((l3 * p3.z + l2 * p2.z + l1 * p1.z) / (l3 + l2 + l1))
            );
        }

        public Triangle(Point aP1, Point aP2, Point aP3, Point aRawN) : this(aP1, aP2, aP3) {
            rawN = aRawN;
        }

        public Point GetNormal() => Settings.GetInstance().UseRawNormals ? rawN : GetPlane().GetNormal();

        public Plane GetPlane() => new Plane(p1, p2, p3);

        public List<Point> GetPoints() => new List<Point>{ p1, p2, p3 };


        private List<Edge> edges = null;

        public List<Edge> GetEdges() {
            if (edges is null) {
                edges = new List<Edge> {
                    new Edge(p1, p2),
                    new Edge(p1, p3),
                    new Edge(p2, p3),
                };
            }

            return edges;
        }

        public float GetPerimeter() => (float)(l1 + l2 + l3);

        public float GetSemiPerimeter() => GetPerimeter() / 2;

        public float GetRadiusOfTheCircumscribedCircle() => (float)(l1 * l2 * l3) / 4 / (float)GetSquare();

        public Point GetCenterOfTheInscribedCircle() {
            var dp1 = (float)(p2 - p3).Magnitude;
            var dp2 = (float)(p1 - p3).Magnitude;
            var dp3 = (float)(p1 - p2).Magnitude;
            var p = dp1 + dp2 + dp3;

            return new Point(
                (p1.x * dp1 + p2.x * dp2 + p3.x * dp3) / p,
                (p2.y * dp1 + p2.y * dp2 + p3.y * dp3) / p,
                (p3.z * dp1 + p2.z * dp2 + p3.z * dp3) / p
            );
        }

        public double GetSquare() {
            var p = GetSemiPerimeter();
            return Math.Sqrt(p * (p - l1) * (p - l2) * (p - l3));
        }

        public static bool operator ==(Triangle a, Triangle b) {
            return a.GetPlane().GetNormal() == b.GetPlane().GetNormal() && (
                a.p1 == b.p1 && a.p2 == b.p2 && a.p3 == b.p3 ||
                a.p1 == b.p1 && a.p2 == b.p3 && a.p3 == b.p2 ||
                a.p1 == b.p2 && a.p2 == b.p1 && a.p3 == b.p3 ||
                a.p1 == b.p2 && a.p2 == b.p3 && a.p3 == b.p1 ||
                a.p1 == b.p3 && a.p2 == b.p1 && a.p3 == b.p2 ||
                a.p1 == b.p3 && a.p2 == b.p2 && a.p3 == b.p1
            );
        }

        public static bool operator !=(Triangle a, Triangle b) => !(a == b);
        public override int GetHashCode() => p1.GetHashCode() + p2.GetHashCode() + p3.GetHashCode();
    }
}
