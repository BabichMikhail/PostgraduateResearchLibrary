using System;
using System.Collections.Generic;
using System.Diagnostics;
using Library.Generic;
using Library.PathFinders;

namespace PathFinders
{
    public class Position {
        public readonly Point originPosition;
        public readonly Point paintDirection;
        public readonly Point surfacePosition;
        public readonly PositionType type;

        public enum PositionType {
            Start,
            Middle,
            Finish,
        }

        public Position(Point aOriginPosition, Point aPaintDirection, Point aSurfacePosition, PositionType aType) {
            originPosition = aOriginPosition;
            paintDirection = aPaintDirection.Normalized;
            surfacePosition = aSurfacePosition;
            type = aType;
            Debug.Assert((paintDirection - (surfacePosition - originPosition).Normalized).Magnitude < 1e-4);
        }
    }

    public enum PathFinderType {
        IntersectionsWithSurfacesPathFinder,
    }

    public interface IPathFinder {
        List<Position> GetPath(ref List<Triangle> triangles);
    }

    public static class PathFinderFactory {
        public static IPathFinder Create(PathFinderType type, float paintRadius, float paintHeight, float paintLateralAllowance, float paintLongitudinalAllowance) {
            IPathFinder result = null;
            switch (type) {
                case PathFinderType.IntersectionsWithSurfacesPathFinder:
                    result = new IntersectionsWithSurfacesPathFinder(paintRadius, paintHeight, paintLateralAllowance, paintLongitudinalAllowance);
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(type), type, null);
            }

            return result;
        }
    }
}
