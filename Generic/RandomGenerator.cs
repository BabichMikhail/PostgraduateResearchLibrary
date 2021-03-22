using System;

namespace Library.Generic {
    public static class RandomGenerator {
        private static Random random = null;

        private static Random GetInstance() {
            return random ?? (random = new Random());
        }

        public static float Float() => (float)GetInstance().NextDouble();
    }
}