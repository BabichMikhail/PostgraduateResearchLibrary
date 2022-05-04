using System;
using System.Collections.Generic;
using Library.Generic;

namespace Library.PathFinders  {
    public class AngleOnPlanePathFinder : IntersectionsWithSurfacesPathFinder {
        private double angle;

        public AngleOnPlanePathFinder(
            float aPaintRadius,
            float aPaintHeight,
            float aAngle,
            float aPaintLateralAllowance,
            float aPaintLongitudinalAllowance,
            float aPaintLineWidth,
            bool aAddExtraParallelPaths
        ) : base(aPaintRadius, aPaintHeight, aPaintLateralAllowance, aPaintLongitudinalAllowance, aPaintLineWidth, aAddExtraParallelPaths) {
            angle = aAngle / 180 * Math.PI;
        }

        public override List<List<Position>> GetPaths(List<Triangle> triangles) {
            var paths = base.GetPaths(triangles);

            var newPaths = new List<List<Position>>();
            foreach (var path in paths) {
                var newPath = new List<Position>();
                foreach (var position in path) {
                    var surfacePoint = position.surfacePoint;
                    var originPoint = position.originPoint;

                    var dy = originPoint.y - surfacePoint.y;
                    var dz = originPoint.z - surfacePoint.z;
                    var newOriginPoint = new Point(
                        originPoint.x,
                        surfacePoint.y + dz * (float)Math.Sin(angle) + dy * (float)Math.Cos(angle),
                        surfacePoint.z + dz * (float)Math.Cos(angle) - dy * (float)Math.Sin(angle)
                    );
                    newPath.Add(new Position(
                        newOriginPoint,
                        (surfacePoint - newOriginPoint).Normalized,
                        surfacePoint,
                        position.type
                    ));
                }

                newPaths.Add(newPath);
            }

            return newPaths;
        }
    }
}