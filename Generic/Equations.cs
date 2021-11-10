using System.Diagnostics;
using System.IO;

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

        public static Matrix QuadraticProgrammingASLE(Matrix r, Matrix s, Matrix g, Matrix h, Matrix a, Matrix b, Matrix w) {
            var pythonProgram =
                "from cvxopt import matrix, solvers\n" +
                // "from qpsolvers import solve_qp\n\n\n" +
                // "def quadprog_solve_qp(P, q, G, h, A, b):\n" +
                // "   qp_G = .5 * (P + P.T)\n" +
                // "   qp_a = -q\n" +
                // "   qp_C = -numpy.vstack([A, G]).T\n" +
                // "   qp_b = -numpy.hstack([b, h])\n" +
                // "   meq = A.shape[0]\n" +
                // "   return quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]\n\n\n" +
                $"r = matrix({r.ToString()})\n" +
                $"s = matrix({s.ToString()})\n" +
                $"g = matrix({g.ToString()})\n" +
                $"h = matrix({h.ToString()})\n" +
                // $"a = matrix({a.ToString()})\n" +
                // $"b = matrix({b.ToString()})\n" +
                $"w = matrix({w.ToString()})\n" +
                "\n" +
                "r = r.trans()\n" +
                "s = s.trans()\n" +
                "g = g.trans()\n" +
                "h = h.trans()\n" +
                // "a = a.trans()\n" +
                // "b = b.trans()\n" +
                "w = w.trans()\n" +
                "\n" +
                "p = r.trans() * w * r\n" +
                "p = p.trans()\n" +
                "q = -r.trans() * w * s\n" +
                // "x = solve_qp(p, q, g, h, a, b)\n" +
                "x = solvers.qp(p, q, g, h)[\"x\"]\n" +
                // "x = solvers.qp(p, q, g, h, a, b)[\"x\"]\n" +
                // "x = quadprog_solve_qp(p, q, g, h, a, b)\n" +
                // "x = solve_qp(P, q, G, h, A, b, lb, ub, solver=solver)\n" +
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
