using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace Library.Generic
{
    public class Matrix {
        private List<List<double>> items;

        public Matrix(List<List<double>> m) {
            items = m;
        }

        public double Get(int row, int column) => items[row][column];

        public int RowCount => items.Count;

        public int ColumnCount => items.Count > 0 ? items.First().Count : 0;

        public double Determinant => GetDet();

        private void CheckIsSquare() {
            if (RowCount != ColumnCount) {
                throw new Exception("Matrix is not square");
            }
        }

        public Matrix Transpose() {
            var newItems = new List<List<double>>();
            var newColumnCount = RowCount;
            var newRowCount = ColumnCount;
            for (var i = 0; i < newRowCount; ++i) {
                var newRow = new List<double>();
                for (var j = 0; j < newColumnCount; ++j) {
                    newRow.Add(items[j][i]);
                }
                newItems.Add(newRow);
            }
            return new Matrix(newItems);
        }

        public static Matrix operator*(Matrix m1, Matrix m2) {
            if (m1.ColumnCount != m2.RowCount) {
                throw new Exception("Unable to multiply matrices");
            }

            var newItems = new List<List<double>>();
            var newRowCount = m1.RowCount;
            var newColumnCount = m2.ColumnCount;
            for (var i = 0; i < newRowCount; ++i) {
                var newRow = new List<double>();
                for (var j = 0; j < newColumnCount; ++j) {
                    var val = 0.0;
                    for (var k = 0; k < m1.ColumnCount; ++k) {
                        val += m1.items[i][k] * m2.items[k][j];
                    }
                    newRow.Add(val);
                }
                newItems.Add(newRow);
            }

            return new Matrix(newItems);
        }

        public static Matrix operator-(Matrix m1, Matrix m2) {
            var newItems = new List<List<double>>();
            for (var i = 0; i < m1.RowCount; ++i) {
                var newRow = new List<double>();
                for (var j = 0; j < m1.ColumnCount; ++j) {
                    newRow.Add(m1.items[i][j] - m2.items[i][j]);
                }

                newItems.Add(newRow);
            }

            return new Matrix(newItems);
        }

        public static bool operator==(Matrix m1, Matrix m2) {
            if (m1 is null && m2 is null) {
                return true;
            }

            if (m1 is null || m2 is null || m1.RowCount != m2.RowCount || m1.ColumnCount != m2.ColumnCount) {
                return false;
            }

            for (var i = 0; i < m1.RowCount; ++i) {
                for (var j = 0; j < m1.ColumnCount; ++j) {
                    if (Math.Abs(m1.items[i][j] - m2.items[i][j]) > double.Epsilon) {
                        return false;
                    }
                }
            }

            return true;
        }

        public static bool operator !=(Matrix m1, Matrix m2) {
            return !(m1 == m2);
        }

        public double GetDet() {
            CheckIsSquare();

            var lu = GetLU();
            var l = lu.Key;
            var u = lu.Value;

            var result = 1.0;
            var n = l.RowCount;
            for (var i = 0; i < n; ++i) {
                result *= l.items[i][i] * u.items[i][i];
            }

            return result;
        }

        // ReSharper disable once InconsistentNaming
        public KeyValuePair<Matrix, Matrix> GetLU() {
            CheckIsSquare();

            var lItems = new List<List<double>>();
            var uItems = new List<List<double>>();

            var n = RowCount;
            for (var i = 0; i < n; ++i) {
                var lRow = new List<double>();
                var uRow = new List<double>();
                for (var j = 0; j < n; ++j) {
                    lRow.Add(i != j ? 0.0 : 1.0);
                    uRow.Add(0.0);
                }

                lItems.Add(lRow);
                uItems.Add(uRow);
            }

            for (var i = 0; i < n; ++i) {
                for (var j = 0; j < n; ++j) {
                    if (i <= j) {
                        var s = 0.0;
                        for (var k = 0; k < i; ++k) {
                            s += lItems[i][k] * uItems[k][j];
                        }

                        uItems[i][j] = items[i][j] - s;
                    }
                    else {
                        var s = 0.0;
                        for (var k = 0; k < j; ++k) {
                            s += lItems[i][k] * uItems[k][j];
                        }

                        lItems[i][j] = (items[i][j] - s) / uItems[j][j];
                    }
                }
            }

            return new KeyValuePair<Matrix, Matrix>(new Matrix(lItems), new Matrix(uItems));
        }

        public Matrix Inverse() {
            CheckIsSquare();

            var n = RowCount;
            var newItems = new List<List<double>>();
            var det = Determinant;
            for (var i = 0; i < n; ++i) {
                var newRow = new List<double>();
                for (var j = 0; j < n; ++j) {
                    var sign = (i + j) % 2 == 0 ? 1 : -1;
                    var subItems = new List<List<double>>();
                    for (var k = 0; k < n; ++k) {
                        if (i != k) {
                            var subRow = new List<double>();
                            for (var p = 0; p < n; ++p) {
                                if (j != p) {
                                    subRow.Add(sign * items[k][p]);
                                }
                            }

                            subItems.Add(subRow);
                        }
                    }

                    newRow.Add(new Matrix(subItems).Determinant / det);
                }

                newItems.Add(newRow);
            }

            return new Matrix(newItems);
        }

        public override string ToString() {
            var result = "[\n";
            foreach (var row in items) {
                var str = "\t[";
                foreach (var element in row) {
                    str += element.ToString("E", System.Globalization.CultureInfo.InvariantCulture) + ",\t";
                }

                result += str.Substring(0, str.Length - 2) + "],\n";
            }

            return result.Substring(0, result.Length - 2) + "\n]";
        }

        public static Matrix FromString(string matrixStr) {
            var newMatrixStr = matrixStr.Substring(matrixStr.IndexOf("[", StringComparison.Ordinal) + 1).Trim();
            var items = new List<List<double>>();
            while (true) {
                var openIdx = newMatrixStr.IndexOf("[", StringComparison.Ordinal);
                if (openIdx == -1) {
                    break;
                }

                var newRow = new List<double>();
                newMatrixStr = newMatrixStr.Substring(openIdx + 1).TrimStart();
                while (true) {
                    var idx = -1;
                    var idx1 = newMatrixStr.IndexOf("]", StringComparison.Ordinal);
                    var idx2 = newMatrixStr.IndexOf(",", StringComparison.Ordinal);
                    if (idx1 == -1) {
                        throw new Exception("Magic");
                    }

                    if (idx2 != -1) {
                        idx = Math.Min(idx1, idx2);
                    }
                    else {
                        idx = idx1;
                    }

                    var str = newMatrixStr.Substring(0, idx).Trim();
                    newMatrixStr = newMatrixStr.Substring(idx + 1).Trim();
                    newRow.Add(double.Parse(str, CultureInfo.InvariantCulture));

                    if (idx2 == -1) {
                        break;
                    }
                }

                items.Add(newRow);
            }

            return new Matrix(items);
        }
    }
}
