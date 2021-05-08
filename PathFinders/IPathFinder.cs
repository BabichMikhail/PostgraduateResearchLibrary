using System;
using System.Collections.Generic;
using Library.Generic;
using Library.PathFinders;

namespace PathFinders
{
    public enum PathFinderType {
        IntersectionsWithSurfacesPathFinder,
    }

    public interface IPathFinder {
        List<Position> GetPath(List<Triangle> triangles);
    }

    public static class PathFinderFactory {
        public static IPathFinder Create(
            PathFinderType type, float paintRadius, float paintHeight, float paintLateralAllowance, float paintLongitudinalAllowance,
            float paintLineWidth
        ) {
            IPathFinder result = null;
            switch (type) {
                case PathFinderType.IntersectionsWithSurfacesPathFinder:
                    result = new IntersectionsWithSurfacesPathFinder(paintRadius, paintHeight, paintLateralAllowance, paintLongitudinalAllowance, paintLineWidth);
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(type), type, null);
            }

            return result;
        }
    }
}
