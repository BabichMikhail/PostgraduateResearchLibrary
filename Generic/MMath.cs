using System;

namespace Library.Generic
{
    public static class MMath {
        public const float PI = (float)Math.PI;

        public static double GetDistance(Point p1, Point p2) => (p1 - p2).Magnitude;

        public static double GetDistance(Plane plane, Edge edge) => Math.Min(plane.GetDistance(edge.p1), plane.GetDistance(edge.p2));

        public static double GetDistance(Edge edge, Point point) => Math.Min((edge.p1 - point).Magnitude, (edge.p2 - point).Magnitude);

        // For disjoint edges only!
        public static double GetDistance(Edge edge1, Edge edge2) => Math.Min(GetDistance(edge1, edge2.p1), GetDistance(edge1, edge2.p2));

        public static double Dot(Point p1, Point p2) => p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;

        public static float Cos(float d) => (float)Math.Cos(d);
        public static float Sin(float d) => (float)Math.Sin(d);
        public static float Round(float d) => (float)Math.Round(d);
    }
}
