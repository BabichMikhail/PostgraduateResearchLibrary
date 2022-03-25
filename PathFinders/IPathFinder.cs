using System;
using System.Collections.Generic;
using Library.Generic;
using Library.PathFinders;

namespace PathFinders  {
    public enum PathFinderType {
        IntersectionsWithSurfacesPathFinder,
        AngleOnPlanePathFinder,
    }

    public interface IPathFinder {
        List<List<Position>> GetPaths(List<Triangle> triangles);
    }

    public static class PathFinderFactory {
        public static IPathFinder Create(
            PathFinderType type, float paintRadius, float paintHeight, float paintAngle, float paintLateralAllowance, float paintLongitudinalAllowance,
            float paintLineWidth, bool addExtraParallelPaths
        ) {
            IPathFinder result = null;
            switch (type) {
                case PathFinderType.IntersectionsWithSurfacesPathFinder:
                    result = new IntersectionsWithSurfacesPathFinder(paintRadius, paintHeight, paintLateralAllowance, paintLongitudinalAllowance, paintLineWidth, addExtraParallelPaths);
                    break;
                case PathFinderType.AngleOnPlanePathFinder:
                    result = new AngleOnPlanePathFinder(paintRadius, paintHeight, paintAngle, paintLateralAllowance, paintLongitudinalAllowance, paintLineWidth, addExtraParallelPaths);
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(type), type, null);
            }

            return result;
        }
    }
}
