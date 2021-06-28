using System;
using System.Collections.Generic;
using System.Linq;
using Library.Generic;

namespace Library.RobotPathBuilder {
    public class RobotPathItem {
        public readonly Position a;
        public readonly Position b;

        public RobotPathItem(Position aFrom, Position aTo) {
            a = aFrom;
            b = aTo;
        }

        public float GetSpeedMultiplier() {
            var originDistance = MMath.GetDistance(a.originPoint, b.originPoint);
            var surfaceDistance = MMath.GetDistance(a.surfacePoint, b.surfacePoint);
            return (float)(originDistance / surfaceDistance);
        }

        public float GetSpeedDecelerationMultiplier(float surfaceSpeed, float maxOriginSpeed) {
            if (float.IsNaN(maxOriginSpeed)) {
                return 1.0f;
            }

            var speed = GetSpeedMultiplier() * surfaceSpeed;
            return Math.Min(speed, maxOriginSpeed) / speed;
        }

        public float GetSpeed(float surfaceSpeed, float maxOriginSpeed) {
            return surfaceSpeed * GetSpeedMultiplier() * GetSpeedDecelerationMultiplier(surfaceSpeed, maxOriginSpeed);
        }
    }

    public class RobotPathProcessor {
        private readonly List<RobotPathItem> pathItems;
        private float speed = 0.0f;
        private float maxSpeed = float.NaN;
        private int currentPathItemIdx = 0;
        private RobotPathItem currentPathItem;
        private float lastMoveDistance = 0.0f;

        public RobotPathProcessor(List<RobotPathItem> aPathItems) {
            pathItems = aPathItems;
            currentPathItem = pathItems[currentPathItemIdx];
        }

        public List<Position> GetRawPath() {
            var result = new List<Position>();
            pathItems.ForEach(x => result.Add(x.a));
            result.Add(pathItems.Last().b);

            return result;
        }

        public void SetSurfaceSpeed(float aSpeed) {
            speed = aSpeed;
        }

        public float GetSurfaceSpeed() {
            return speed;
        }

        public void SetMaxOriginSpeed(float aMaxSpeed) {
            maxSpeed = aMaxSpeed;
        }

        public float GetMaxOriginSpeed() {
            return maxSpeed;
        }

        public float GetCurrentOriginSpeed() {
            var result = 0.0f;
            if (currentPathItemIdx < pathItems.Count) {
                result = speed * pathItems[currentPathItemIdx].GetSpeedMultiplier() * pathItems[currentPathItemIdx].GetSpeedDecelerationMultiplier(speed, maxSpeed);
            }

            return result;
        }

        public float GetAcceleration(int positionIndex) {
            var result = 0.0f;
            // TODO
            // if (positionIndex < pathItems.Count - 1) {
            //     var p1 = pathItems[positionIndex];
            //     var p2 = pathItems[positionIndex + 1];
            //
            //     var v1 = p1.speedMultiplier * p1.speedDecelerationMultiplier;
            //     var v2 = p2.speedMultiplier * p2.speedDecelerationMultiplier;
            //     var s = (float)MMath.GetDistance(p1.a.originPoint, p2.b.originPoint);
            //
            //     var t = 2 * s / (v1 + v2);
            //     result = (v2 - v1) / t;
            // }

            return result;
        }

        public float GetCurrentAcceleration() {
            return GetAcceleration(currentPathItemIdx);
        }

        public float GetLastMoveDistance() {
            return lastMoveDistance;
        }

        public RobotPathItem GetCurrentRobotPathItem() {
            return currentPathItem;
        }

        public List<RobotPathItem> GetRobotPathItems() {
            return pathItems;
        }

        public RobotPathItem Move(float time) {
            lastMoveDistance = 0.0f;
            var distance = speed * time;
            while (distance > 0 && currentPathItemIdx < pathItems.Count) {
                var length = (float)MMath.GetDistance(currentPathItem.a.originPoint, currentPathItem.b.originPoint);

                var speedMultiplier = currentPathItem.GetSpeedMultiplier();
                var speedDecelerationMultiplier = currentPathItem.GetSpeedDecelerationMultiplier(speed, float.NaN);
                var speedCoefficient = speedMultiplier * speedDecelerationMultiplier;

                if (distance * speedCoefficient >= length) {
                    var moveDistance = length / speedCoefficient;

                    lastMoveDistance += moveDistance;
                    distance -= moveDistance;

                    ++currentPathItemIdx;
                    if (currentPathItemIdx < pathItems.Count) {
                        currentPathItem = pathItems[currentPathItemIdx];
                    }
                }
                else {
                    lastMoveDistance += distance * speedCoefficient;

                    var k = distance * speedCoefficient / length;
                    var a = currentPathItem.a;
                    var b = currentPathItem.b;

                    var newOriginPoint = a.originPoint + (b.originPoint - a.originPoint) * k;
                    var newSurfacePoint = a.surfacePoint + (b.surfacePoint - a.surfacePoint) * k;
                    var newDirection = a.paintDirection + (b.paintDirection - a.paintDirection) * k;
                    var newPosition = new Position(newOriginPoint, newDirection, newSurfacePoint, Position.PositionType.Middle);

                    currentPathItem = new RobotPathItem(newPosition, pathItems[currentPathItemIdx].b);
                    distance = 0;
                }
            }

            return currentPathItem;
        }

        public void Reset() {
            currentPathItemIdx = 0;
            currentPathItem = pathItems[currentPathItemIdx];
        }

        public bool IsFinished() {
            return currentPathItemIdx == pathItems.Count;
        }
    }

    public static class RobotPathProcessorBuilder {
        public static List<RobotPathItem> BuildRobotPathItems(List<Position> positions) {
            var robotPathItems = new List<RobotPathItem>();
            for (var i = 0; i < positions.Count - 1; ++i) {
                if (positions[i].originPoint == positions[i + 1].originPoint) {
                    throw new Exception("Illegal positions");
                }

                robotPathItems.Add(new RobotPathItem(positions[i], positions[i + 1]));
            }

            return robotPathItems;
        }

        public static List<RobotPathProcessor> Build(List<List<Position>> paths, float surfaceSpeed, float maxOriginSpeed) {
            var result = new List<RobotPathProcessor>();
            paths.ForEach(x => result.Add(Build(BuildRobotPathItems(x), surfaceSpeed, maxOriginSpeed)));

            return result;
        }

        public static RobotPathProcessor Build(List<RobotPathItem> pathItems, float surfaceSpeed, float maxOriginSpeed) {
            var rpp = new RobotPathProcessor(pathItems);
            rpp.SetSurfaceSpeed(surfaceSpeed);
            rpp.SetMaxOriginSpeed(maxOriginSpeed);

            return rpp;
        }
    }
}
