using System;
using System.Collections.Generic;
using System.Diagnostics;
using Library.Generic;

namespace Library.PathApproximation
{
    class Equation3 : BaseEquation {
        private readonly double a;
        private readonly double b;
        private readonly double c;
        private readonly double d;

        public Equation3(double tFrom, double tTo, double a, double b, double c, double d) : base(tFrom, tTo) {
            this.a = a;
            this.b = b;
            this.c = c;
            this.d = d;
        }

        public override double GetValue(double t) => t * t * t * a + t * t * b + t * c + d;
    }

    public class ThirdOrderApproximation : BaseApproximation {
        public ThirdOrderApproximation(bool useAvgNormals, bool reNormalizeNormals) : base(useAvgNormals, reNormalizeNormals) {}

        protected override int GetTXLength() => 4;

        protected override IEquation GetEquation(List<tx> txs) {
            return GetEquation(
                txs[0].t, txs[0].x,
                txs[1].t, txs[1].x,
                txs[2].t, txs[2].x,
                txs[3].t, txs[3].x
            );
        }

        private Equation3 GetEquation(double t1, double x1, double t2, double x2, double t3, double x3, double t4, double x4) {
            var a11 = t1 * t1 * t1;
            var a12 = t1 * t1;
            var a13 = t1;
            var a14 = 1;
            var a21 = t2 * t2  *t2;
            var a22 = t2 * t2;
            var a23 = t2;
            var a24 = 1;
            var a31 = t3 * t3 * t3;
            var a32 = t3 * t3;
            var a33 = t3;
            var a34 = 1;
            var a41 = t4 * t4 * t4;
            var a42 = t4 * t4;
            var a43 = t4;
            var a44 = 1;

            var d = MMath.GetDeterminant(new List<List<double>>
                {new List<double>{a11, a12, a13, a14}, new List<double>{a21, a22, a23, a24}, new List<double>{a31, a32, a33, a34}, new List<double>{a41, a42, a43, a44}}
            );
            var d1 = MMath.GetDeterminant(new List<List<double>>
                {new List<double>{x1, a12, a13, a14}, new List<double>{x2, a22, a23, a24}, new List<double>{x3, a32, a33, a34}, new List<double>{x4, a42, a43, a44}}
            );
            var d2 = MMath.GetDeterminant(new List<List<double>>
                {new List<double>{a11, x1, a13, a14}, new List<double>{a21, x2, a23, a24}, new List<double>{a31, x3, a33, a34}, new List<double>{a41, x4, a43, a44}}
            );
            var d3 = MMath.GetDeterminant(new List<List<double>>
                {new List<double>{a11, a12, x1, a14}, new List<double>{a21, a22, x2, a24}, new List<double>{a31, a32, x3, a34}, new List<double>{a41, a42, x4, a44}}
            );
            var d4 = MMath.GetDeterminant(new List<List<double>>
                {new List<double>{a11, a12, a13, x1}, new List<double>{a21, a22, a23, x2}, new List<double>{a31, a32, a33, x3}, new List<double>{a41, a42, a43, x4}}
            );

            var a1 = d1 / d;
            var a2 = d2 / d;
            var a3 = d3 / d;
            var a4 = d4 / d;

            var tMin = t1;
            var tMax = t1;
            foreach (var t in new List<double>{t1, t2, t3, t4}) {
                if (tMin > t) {
                    tMin = t;
                }

                if (tMax < t) {
                    tMax = t;
                }
            }

            var e = new Equation3(tMin, tMax, a1, a2, a3, a4);

            var e1 = e.GetValue(t1);
            if (Math.Abs(e1 - x1) > 5) {
                Debug.Assert(false);
            }

            var e2 = e.GetValue(t2);
            if (Math.Abs(e2 - x2) > 5) {
                Debug.Assert(false);
            }

            var e3 = e.GetValue(t3);
            if (Math.Abs(e3 - x3) > 5) {
                Debug.Assert(false);
            }

            var e4 = e.GetValue(t4);
            if (Math.Abs(e4 - x4) > 5) {
                Debug.Assert(false);
            }

            return e;
        }
    }
}
