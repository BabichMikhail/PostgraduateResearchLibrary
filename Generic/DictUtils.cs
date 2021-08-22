using System.Collections.Generic;

namespace Library.Generic
{
    public static class DictUtils {
        public static void FillValueIfNotExists<TK, TV>(Dictionary<TK, TV> dict, TK k, TV defaultValue) {
            if (!dict.ContainsKey(k)) {
                dict.Add(k, defaultValue);
            }
        }

        public static void SumValue<TK>(Dictionary<TK, double> dict, TK k, double value) {
            FillValueIfNotExists(dict, k, 0.0);
            dict[k] += value;
        }
    }
}