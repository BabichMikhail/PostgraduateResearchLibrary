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
