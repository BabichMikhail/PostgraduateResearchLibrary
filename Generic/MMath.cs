using System;
using System.Collections.Generic;

namespace Library.Generic
{
    public static class MMath {
        public const float PI = (float)Math.PI;
        public const float E = (float)Math.E;

        public static double GetDistance(Point p1, Point p2) => (p1 - p2).Magnitude;

        public static double GetDistance(Plane plane, Point p) => plane.GetDistance(p);

        public static double GetDistance(Plane plane, Edge edge) => Math.Min(GetDistance(plane, edge.p1), GetDistance(plane, edge.p2));

        public static double GetDistance(Plane p1, Plane p2) => GetDistance(p1, p2.GetSomePoint());

        public static double GetDistance(Edge edge, Point point) => Math.Min((edge.p1 - point).Magnitude, (edge.p2 - point).Magnitude);

        // For disjoint edges only!
        public static double GetDistance(Edge edge1, Edge edge2) => Math.Min(GetDistance(edge1, edge2.p1), GetDistance(edge1, edge2.p2));

        public static double GetDistance(Triangle t, Point p) => Math.Min(Math.Min(GetDistance(t.p1, p), GetDistance(t.p2, p)), GetDistance(t.p3, p));

        public static Point Intersect(Plane p, Line l, bool usingExperimentSearch) {
            // its experiment method:
            if (usingExperimentSearch) {
                var p1 = l.p1;
                var p2 = l.p2;
                var pm = (p1 + p2) / 2.0f;

                var dst1 = GetDistance(p, p1);
                var dst2 = GetDistance(p, p2);

                var a = GetDistance(p, p1);
                var b = GetDistance(p, p2);
                var c = GetDistance(p, pm);
                var d = 2 * c;
                var e = a + b - 2 * c;
                if (Math.Abs(dst1 + dst2 - 2 * GetDistance(p, pm)) > 1e-2 || dst1 < 1e-2 || dst2 < 1e-2) {
                    var dm = GetDistance(p, pm);
                    while (dm > 1e-2) {
                        var newP1 = pm;
                        var newP2 = p1;
                        if (GetDistance(p, p1) > GetDistance(p, p2)) {
                            newP2 = p2;
                        }

                        p1 = newP1;
                        p2 = newP2;
                        pm = (p1 + p2) / 2;
                        dm = GetDistance(p, pm);

                        if (GetDistance(p1, p2) < 1e-2 && dm > 1e-2) {
                            // too large calculation error;
                            return null;
                        }
                    }

                    return pm;
                }

                return null;
            }

            // plane:
            // ax + by + cz + d = 0;
            // line:
            // p0y * x - p0x * y           + (p0x * y0 - p0y * x0) = 0
            // p0z * x           - p0x * z + (p0x * z0 - p0z * x0) = 0

            var a11 = p.a;
            var a12 = p.b;
            var a13 = p.c;
            var b1 = -p.d;
            var a21 = l.p0Y;
            var a22 = -l.p0X;
            var a23 = 0;
            var b2 = -(l.p0X * l.y0 - l.p0Y * l.x0);
            var a31 = l.p0Z;
            var a32 = 0;
            var a33 = -l.p0X;
            var b3 = -(l.p0X * l.z0 - l.p0Z * l.x0);

            var d0 = GetDeterminant(new List<List<double>> {
                new List<double>{a11, a12, a13},
                new List<double>{a21, a22, a23},
                new List<double>{a31, a32, a33},
            });
            if (Math.Abs(d0) < 1e-6) {
                return null;
            }

            var d1 = GetDeterminant(new List<List<double>> {
                new List<double>{b1, a12, a13},
                new List<double>{b2, a22, a23},
                new List<double>{b3, a32, a33},
            });
            var d2 = GetDeterminant(new List<List<double>> {
                new List<double>{a11, b1, a13},
                new List<double>{a21, b2, a23},
                new List<double>{a31, b3, a33},
            });
            var d3 = GetDeterminant(new List<List<double>> {
                new List<double>{a11, a12, b1},
                new List<double>{a21, a22, b2},
                new List<double>{a31, a22, b3},
            });

            var x = d1 / d0;
            var y = d2 / d0;
            var z = d3 / d0;

            if (
                x < l.p1.x && x < l.p2.x || x > l.p1.x && x > l.p2.x ||
                y < l.p1.y && y < l.p2.y || y > l.p1.y && y > l.p2.y ||
                z < l.p1.z && z < l.p2.z || z > l.p1.z && z > l.p2.z
            ) {
                return null;
            }

            return new Point((float)x, (float)y, (float)z);
        }

        public static float Dot(Point p1, Point p2) => p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;

        public static float Cos(float d) => (float)Math.Cos(d);
        public static float Sin(float d) => (float)Math.Sin(d);
        public static float Acos(float d) => (float)Math.Acos(d);
        public static float Tan(float d) => (float)Math.Tan(d);
        public static float Round(float d) => (float)Math.Round(d);
        public static float Sqrt(float x) => (float)Math.Sqrt(x);
        public static float Pow(float x, float y) => (float)Math.Pow(x, y);
        public static float Log(float x) => (float)Math.Log(x);

        public static float GetRandomGaussian() {
            float v1, s;
            do {
                v1 = 2.0f * RandomGenerator.Float() - 1.0f;
                var v2 = 2.0f * RandomGenerator.Float() - 1.0f;
                s = v1 * v1 + v2 * v2;
            } while (s >= 1.0f || s == 0.0f);

            s = Sqrt(-2.0f * Log(s) / s);

            return v1 * s;
        }

        public static float GetRandomGaussian(float mean, float standardDeviation) {
            return mean + GetRandomGaussian() * standardDeviation;
        }

        public static float GetNormalDistributionProbabilityDensity(float x, float mean, float standardDeviation) {
            return Pow(E, -Pow(x - mean, 2) / (2 * Pow(standardDeviation, 2))) / (standardDeviation * Sqrt(2 * PI));
        }

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
