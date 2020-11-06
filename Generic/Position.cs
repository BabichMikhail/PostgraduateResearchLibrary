using System.Diagnostics;

namespace Library.Generic
{
    public class Position {
        public readonly Point originPoint;
        public readonly Point paintDirection;
        public readonly Point surfacePoint;
        public readonly PositionType type;

        public enum PositionType {
            Start,
            Middle,
            Finish,
        }

        public Position(Point aOriginPoint, Point aPaintDirection, Point aSurfacePoint, PositionType aType) {
            originPoint = aOriginPoint;
            paintDirection = aPaintDirection.Normalized;
            surfacePoint = aSurfacePoint;
            type = aType;
            Debug.Assert((paintDirection - (surfacePoint - originPoint).Normalized).Magnitude < 1e-4);
        }
    }
}
