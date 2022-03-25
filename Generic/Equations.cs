using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using Debug = UnityEngine.Debug;

namespace Library.Generic
{
    public static class Equations {
        private static string RunCmd(string cmd, string args){
            var start = new ProcessStartInfo {
                FileName = "G:\\Projects\\3d-scan\\venv\\Scripts\\python.exe",
                Arguments = $"{cmd} {args}",
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
            };

            var process = Process.Start(start);
            var reader = process.StandardOutput;
            var result = reader.ReadToEnd();
            var error = process.StandardError.ReadToEnd();
            if (error.Length > 0) {
                UnityEngine.Debug.Log(error);
            }
            return result;
        }

        // ax = b => x = (transpose(a) * a)^(-1) * transpose(a) * b
        public static Matrix LessEquations(Matrix a, Matrix b) {
            var pythonProgram =
                "import numpy\n" +
                $"a = numpy.matrix({a.ToString()})\n" +
                $"b = numpy.matrix({b.ToString()})\n" +
                "at = a.transpose()\n" +
                "at_a = at * a\n" +
                "at_a_inv = numpy.linalg.inv(at_a)\n" +
                "at_a_inv_at = at_a_inv * at\n" +
                "x = at_a_inv_at * b\n" +
                "print(x)\n";
            var filename = Path.GetTempFileName();
            File.WriteAllText(filename, pythonProgram);
            var pyX = Matrix.FromString(RunCmd(filename, ""));
            return pyX;
        }

        public static Matrix QuadraticProgramming(Matrix p, Matrix q, Matrix g, Matrix h, Matrix a, Matrix b) {
            var pythonProgram =
                "import numpy\n" +
                "from qpsolvers import solve_qp\n" +
                // "import quadprog\n\n\n" +
                // "def quadprog_solve_qp(P, q, G, h, A, b):\n" +
                // "   qp_G = .5 * (P + P.T)\n" +
                // "   qp_a = -q\n" +
                // "   qp_C = -numpy.vstack([A, G]).T\n" +
                // "   qp_b = -numpy.hstack([b, h])\n" +
                // "   meq = A.shape[0]\n" +
                // "   return quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]\n\n\n" +
                $"p = numpy.matrix({p.ToString()})\n" +
                $"q = numpy.matrix({q.ToString()})\n" +
                $"g = numpy.matrix({g.ToString()})\n" +
                $"h = numpy.matrix({h.ToString()})\n" +
                $"a = numpy.matrix({a.ToString()})\n" +
                $"b = numpy.matrix({b.ToString()})\n" +
                "x = solve_qp(p, q, g, h, a, b)\n" +
                // "x = quadprog_solve_qp(p, q, g, h, a, b)\n" +
                "print(x)\n";
            var filename = Path.GetTempFileName();
            File.WriteAllText(filename, pythonProgram);
            var pyX = Matrix.FromString(RunCmd(filename, ""));
            return pyX;
        }

        private static string ToString(List<double> values) {
            var str = "[";
            foreach (var value in values) {
                str += value.ToString("E", CultureInfo.InvariantCulture) + ", ";
            }

            return str + "]";
        }

        private static string ToString(double value) {
            return value.ToString("E", CultureInfo.InvariantCulture);
        }

        public static Matrix QuadraticProgrammingASLEV2(
            Matrix r, Matrix s, Matrix w, List<double> speeds, List<double> distances, double minSpeed, double maxSpeed,
            double maxAcceleration, bool useLimits
        ) {
            var iterationLimit = "1000";
            var limitsCondition = "";
            if (!useLimits) {
                limitsCondition = "False and ";
                iterationLimit = "1";
            }
            var pythonProgram = $@"
from cvxopt import matrix, solvers
import math


solvers.options['glpk'] = dict(msg_lev='GLP_MSG_OFF')
solvers.options['show_progress'] = False


min_speed = {ToString(minSpeed)}
max_speed = {ToString(maxSpeed)}
max_acceleration = {ToString(maxAcceleration)}


def get_acceleration(v1, d1, v2, d2):
    return (v2 * v2 - v1 * v1) / (d1 + d2)


def get_limit(v1, w1, v2, w2, acceleration):
    if abs(acceleration) < 0.00001:
        acceleration = 0.00001
    return abs(w1 / v1 - w2 / v2) * (max_acceleration / acceleration)


speeds = {ToString(speeds)}
distances = {ToString(distances)}

weights = []
for i in range(len(speeds)):
    weights.append(1.0)

iteration = 0.0

r = matrix({r.ToString()})
s = matrix({s.ToString()})
w = matrix({w.ToString()})

r = r.trans()
s = s.trans()
w = w.trans()

p = r.trans() * w * r
p = p.trans()
q = -r.trans() * w * s

usedLimitsIndexes = {{}}
prevLimits = []
while True:
    if iteration > {iterationLimit}:
        break
    iteration += 1.0
    limits = []
    for i in range(len(speeds) - 1):
        acceleration = get_acceleration(speeds[i] / weights[i], distances[i], speeds[i + 1] / weights[i + 1], distances[i + 1])
        newLimit = get_limit(speeds[i], weights[i], speeds[i + 1], weights[i + 1], acceleration)
        limits.append(newLimit)
        #if len(prevLimits) > 0 and abs(limits[i]) > abs(prevLimits[i]):
        #    limits[i] = prevLimits[i]
        if {limitsCondition} abs(acceleration) > max_acceleration and not i in usedLimitsIndexes.keys():
            usedLimitsIndexes[i] = True

    prevLimits = limits
    gm = []
    hm = []

    for i in range(len(speeds) - 1):
        if i in usedLimitsIndexes.keys():
            newGRow1 = []
            #newGRow2 = []
            for j in range(i):
                newGRow1.append(0.0)
                #newGRow2.append(0.0)

            v1 = speeds[i]
            v2 = speeds[i + 1]
            c = 1.0
            if limits[i] < 0:
                c = -1.0

            newGRow1.append(c / v1)
            newGRow1.append(-c / v2)
            #newGRow2.append(-c / v1)
            #newGRow2.append(c / v2)

            for j in range(len(speeds) - 2 - i):
                newGRow1.append(0.0)
                #newGRow2.append(0.0)

            gm.append(newGRow1)
            #gm.append(newGRow2)
            #hm.append([abs(limits[i])])
            hm.append([abs(limits[i])])

    for i in range(len(speeds)):
        newGRow = []
        for j in range(i):
            newGRow.append(0.0)

        newGRow.append(1.0)

        for j in range(len(speeds) - 1 - i):
            newGRow.append(0.0)

        #gm.append(newGRow)
        #hm.append([speeds[i] / min_speed])

    for i in range(len(speeds)):
        newGRow = []
        for j in range(i):
            newGRow.append(0.0)
        
        newGRow.append(-1.0)
        
        for j in range(len(speeds) - 1 - i):
            newGRow.append(0.0)

        gm.append(newGRow)
        hm.append([-speeds[i] / max_speed])

    g = matrix(gm)
    h = matrix(hm)

    g = g.trans()
    h = h.trans()

    weights = solvers.qp(p, q, g, h)['x']

    ok = True
    for i in range(len(speeds) - 1):
        acceleration = get_acceleration(speeds[i] / weights[i], distances[i], speeds[i + 1] / weights[i + 1], distances[i + 1])
        ok = ok and (abs(acceleration) <= max_acceleration)

    if ok:
        break

print('[')
print(weights)
print(']')
";
            var filename = Path.GetTempFileName();
            Debug.Log(filename);
            File.WriteAllText(filename, pythonProgram);
            var pyX = Matrix.FromString(RunCmd(filename, ""));
            return pyX;
        }

        public static Matrix QuadraticProgrammingASLE(Matrix r, Matrix s, Matrix g, Matrix h, Matrix w) {
            var pythonProgram =
                "from cvxopt import matrix, solvers\n" +

                $"r = matrix({r.ToString()})\n" +
                $"s = matrix({s.ToString()})\n" +
                $"g = matrix({g.ToString()})\n" +
                $"h = matrix({h.ToString()})\n" +
                $"w = matrix({w.ToString()})\n" +
                "\n" +
                "r = r.trans()\n" +
                "s = s.trans()\n" +
                "g = g.trans()\n" +
                "h = h.trans()\n" +
                "w = w.trans()\n" +
                "\n" +
                "p = r.trans() * w * r\n" +
                "p = p.trans()\n" +
                "q = -r.trans() * w * s\n" +
                "x = solvers.qp(p, q, g, h)[\"x\"]\n" +
                "print(\"[\")\n" +
                "print(x)\n" +
                "print(\"]\")\n";
            var filename = Path.GetTempFileName();
            File.WriteAllText(filename, pythonProgram);
            var pyX = Matrix.FromString(RunCmd(filename, ""));
            return pyX;
        }
    }
}
