using System;
using System.Collections.Generic;
using System.Diagnostics;
using Library.Generic;

namespace Library.PathApproximation
{
    public class Equation1 : BaseEquation {
        private readonly double a;
        private readonly double b;
        private readonly double c;

        public Equation1(double tFrom, double tTo, double a, double b) : base(tFrom, tTo) {
            this.a = a;
            this.b = b;
        }

        public override double GetValue(double t) => t * a + b;
    }

    public class LinearApproximation : BaseApproximation {
        public LinearApproximation(bool useAvgNormals) : base(useAvgNormals) {}

        protected override int GetTXLength() => 2;

        protected override IEquation GetEquation(List<tx> txs) {
            return GetEquation(txs[0].t, txs[0].x, txs[1].t, txs[1].x);
        }

        private IEquation GetEquation(double t1, double x1, double t2, double x2) {
            var a11 = t1;
            var a12 = 1;
            var a21 = t2;
            var a22 = 1;

            var d = MMath.GetDeterminant(new List<List<double>>
                {new List<double>{a11, a12}, new List<double>{a21, a22}}
            );
            var d1 = MMath.GetDeterminant(new List<List<double>>
                {new List<double>{x1, a12}, new List<double>{x2, a22}}
            );
            var d2 = MMath.GetDeterminant(new List<List<double>>
                {new List<double>{a11, x1}, new List<double>{a21, x2}}
            );

            var a1 = d1 / d;
            var a2 = d2 / d;

            var tMin = t1;
            var tMax = t1;
            foreach (var t in new List<double>{t1, t2}) {
                if (tMin > t) {
                    tMin = t;
                }

                if (tMax < t) {
                    tMax = t;
                }
            }

            var e = new Equation1(tMin, tMax, a1, a2);

            var e1 = e.GetValue(t1);
            if (Math.Abs(e1 - x1) > 1) {
                Debug.Assert(false);
            }

            var e2 = e.GetValue(t2);
            if (Math.Abs(e2 - x2) > 1) {
                Debug.Assert(false);
            }

            return e;
        }
    }
}
