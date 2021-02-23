using System;
using System.Collections.Generic;

namespace Library.Generic
{
    public static class MMath {
        public const float PI = (float)Math.PI;

        public static double GetDistance(Point p1, Point p2) => (p1 - p2).Magnitude;

        public static double GetDistance(Plane plane, Point p) => plane.GetDistance(p);

        public static double GetDistance(Plane plane, Edge edge) => Math.Min(plane.GetDistance(edge.p1), plane.GetDistance(edge.p2));

        public static double GetDistance(Edge edge, Point point) => Math.Min((edge.p1 - point).Magnitude, (edge.p2 - point).Magnitude);

        // For disjoint edges only!
        public static double GetDistance(Edge edge1, Edge edge2) => Math.Min(GetDistance(edge1, edge2.p1), GetDistance(edge1, edge2.p2));

        public static double Dot(Point p1, Point p2) => p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;

        public static float Cos(float d) => (float)Math.Cos(d);
        public static float Sin(float d) => (float)Math.Sin(d);
        public static float Round(float d) => (float)Math.Round(d);

        public static double GetDeterminant(List<List<double>> c) {
            if (c.Count > 2) {
                var result = 0.0;
                for (var i = 0; i < c.Count; ++i) {
                    var m = new List<List<double>>();
                    for (var j = 1; j < c.Count; ++j) {
                        var line = new List<double>();
                        for (var k = 0; k < c.Count; ++k) {
                            if (k != i) {
                                line.Add(c[j][k]);
                            }
                        }

                        m.Add(line);
                    }

                    result += c[0][i] * (float) Math.Pow(-1, i) * GetDeterminant(m);
                }

                return result;
            }

            if (c.Count == 2) {
                return c[0][0] * c[1][1] - c[0][1] * c[1][0];
            }

            if (c.Count == 1) {
                return c[0][0];
            }

            throw new Exception("Not implemented");
        }
    }
}
