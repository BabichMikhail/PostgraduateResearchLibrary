using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace Library.PathApproximation
{
    class BezierEquation : BaseEquation {
        private readonly List<double> xs;
        private readonly List<double> cs;

        public BezierEquation(double tFrom, double tTo, List<double> xs) : base(tFrom, tTo) {
            this.xs = xs;

            cs = new List<double> {1};
            for (var i = 1; i < xs.Count; ++i) {
                var newCs = new List<double>{cs[0]};
                for (var j = 0; j < cs.Count - 1; ++j) {
                    newCs.Add(cs[j] + cs[j + 1]);
                }

                newCs.Add(cs.Last());
                cs = newCs;
            }
        }

        public override double GetValue(double t) {
            Debug.Assert(tFrom <= t && t <= tTo);
            var lt = (t - tFrom) / (tTo - tFrom);

            var value = 0.0;
            for (var i = 0; i < cs.Count; ++i) {
                value += Math.Pow(1 - lt, cs.Count - 1 - i) * Math.Pow(lt, i) * cs[i] * xs[i];
            }

            return value;
        }
    }

    public abstract class BezierApproximation : BaseApproximation {
        private readonly int pow;

        protected BezierApproximation(bool useAvgNormals, bool reNormalizeNormals, int pow) : base(useAvgNormals, reNormalizeNormals) {
            this.pow = pow;
        }

        protected override int GetTXLength() => pow;

        protected override bool MustUseMaxAvailableTxLength() => true;

        protected override IEquation GetEquation(List<tx> txs) {
            var tMin = txs[0].t;
            var tMax = txs[0].t;
            var xs = new List<double>();
            foreach (var tx in txs) {
                tMin = Math.Min(tx.t, tMin);
                tMax = Math.Max(tx.t, tMax);
                xs.Add(tx.x);
            }

            return new BezierEquation(tMin, tMax, xs);
        }

    }

    public class Bezier2Approximation : BezierApproximation {
        public Bezier2Approximation(bool useAvgNormals, bool reNormalizerNormals) : base(useAvgNormals, reNormalizerNormals, 2) {}
    }

    public class Bezier3Approximation : BezierApproximation {
        public Bezier3Approximation(bool useAvgNormals, bool reNormalizerNormals) : base(useAvgNormals, reNormalizerNormals, 3) {}
    }

    public class Bezier4Approximation : BezierApproximation {
        public Bezier4Approximation(bool useAvgNormals, bool reNormalizerNormals) : base(useAvgNormals, reNormalizerNormals, 4) {}
    }

    public class Bezier5Approximation : BezierApproximation {
        public Bezier5Approximation(bool useAvgNormals, bool reNormalizerNormals) : base(useAvgNormals, reNormalizerNormals, 5) {}
    }

    public class Bezier20Approximation : BezierApproximation {
        public Bezier20Approximation(bool useAvgNormals, bool reNormalizerNormals) : base(useAvgNormals, reNormalizerNormals, 20) {}
    }

    public class BezierNApproximation : BezierApproximation {
        public BezierNApproximation(bool useAvgNormals, bool reNormalizeNormals, int pow) : base(useAvgNormals, reNormalizeNormals, pow) {}
    }
}
