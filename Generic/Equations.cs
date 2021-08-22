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
    }
}
