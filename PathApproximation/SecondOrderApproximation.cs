using System;
using System.Collections.Generic;
using System.Diagnostics;
using Library.Generic;

namespace Library.PathApproximation
{
    class Equation2 : BaseEquation {
        private readonly double a;
        private readonly double b;
        private readonly double c;

        public Equation2(double tFrom, double tTo, double a, double b, double c) : base(tFrom, tTo) {
            this.a = a;
            this.b = b;
            this.c = c;
        }

        public override double GetValue(double t) => t * t * a + t * b + c;
    }

    public class SecondOrderApproximation : BaseApproximation {
        public SecondOrderApproximation(bool useAvgNormals) : base(useAvgNormals) {}

        protected override int GetTXLength() => 3;

        protected override IEquation GetEquation(List<tx> txs) {
            return GetEquation(
                txs[0].t, txs[0].x,
                txs[1].t, txs[1].x,
                txs[2].t, txs[2].x
            );
        }

        private Equation2 GetEquation(double t1, double x1, double t2, double x2, double t3, double x3) {
            var a11 = t1 * t1;
            var a12 = t1;
            var a13 = 1;
            var a21 = t2 * t2;
            var a22 = t2;
            var a23 = 1;
            var a31 = t3 * t3;
            var a32 = t3;
            var a33 = 1;

            var d = MMath.GetDeterminant(new List<List<double>>
                {new List<double>{a11, a12, a13}, new List<double>{a21, a22, a23}, new List<double>{a31, a32, a33}}
            );
            var d1 = MMath.GetDeterminant(new List<List<double>>
                {new List<double>{x1, a12, a13}, new List<double>{x2, a22, a23}, new List<double>{x3, a32, a33}}
            );
            var d2 = MMath.GetDeterminant(new List<List<double>>
                {new List<double>{a11, x1, a13}, new List<double>{a21, x2, a23}, new List<double>{a31, x3, a33}}
            );
            var d3 = MMath.GetDeterminant(new List<List<double>>
                {new List<double>{a11, a12, x1}, new List<double>{a21, a22, x2}, new List<double>{a31, a32, x3}}
            );

            var a1 = d1 / d;
            var a2 = d2 / d;
            var a3 = d3 / d;

            var tMin = t1;
            var tMax = t1;
            foreach (var t in new List<double>{t1, t2, t3}) {
                if (tMin > t) {
                    tMin = t;
                }

                if (tMax < t) {
                    tMax = t;
                }
            }

            var e = new Equation2(tMin, tMax, a1, a2, a3);

            var e1 = e.GetValue(t1);
            if (Math.Abs(e1 - x1) > 1) {
                Debug.Assert(false);
            }

            var e2 = e.GetValue(t2);
            if (Math.Abs(e2 - x2) > 1) {
                Debug.Assert(false);
            }

            var e3 = e.GetValue(t3);
            if (Math.Abs(e3 - x3) > 1) {
                Debug.Assert(false);
            }

            return e;
        }
    }
}
