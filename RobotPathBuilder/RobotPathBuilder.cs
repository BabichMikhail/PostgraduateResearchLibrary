using System;
using System.Collections.Generic;
using Library.Generic;

namespace Library.RobotPathBuilder {
    public class RobotPosition {
        public readonly Point point;
        public readonly Point direction;
        public readonly float speedMultiplier;
        public readonly Position position;
        public readonly float speedDecelerationMultiplier;

        public RobotPosition(Point aPoint, Point aDirection, Position aPosition, float aSpeedMultiplier, float aSpeedDecelerationMultiplier) {
            point = aPoint;
            direction = aDirection;
            position = aPosition;
            speedMultiplier = aSpeedMultiplier;
            speedDecelerationMultiplier = aSpeedDecelerationMultiplier;
        }
    }

    public class RobotPathItem {
        public readonly Position position;
        public readonly float acceleration;
        public readonly float speed;

        public RobotPathItem(Position aPosition, float aAcceleration, float aSpeed) {
            position = aPosition;
            acceleration = aAcceleration;
            speed = aSpeed;
        }
    }

    public class RobotPath {
        public readonly List<RobotPathItem> items;
        public readonly float paintSpeed;

        public RobotPath(List<RobotPathItem> aItems, float aPaintSpeed) {
            items = aItems;
            paintSpeed = aPaintSpeed;
        }
    }

    public class RobotPathProcessor {
        private readonly List<RobotPosition> positions;
        private float speed = 0.0f;
        private int currentPos = 0;
        private RobotPosition currentRobotPosition = null;
        private float lastMoveDistance = 0.0f;

        public RobotPathProcessor(List<RobotPosition> aPositions) {
            positions = aPositions;
            currentRobotPosition = positions[0];
        }

        public void SetBaseSpeed(float aSpeed) {
            speed = aSpeed;
        }

        public float GetCurrentSpeed() {
            var result = 0.0f;
            if (currentPos < positions.Count - 1) {
                result = speed * currentRobotPosition.speedMultiplier * currentRobotPosition.speedDecelerationMultiplier;
            }

            return result;
        }

        public float GetAcceleration(int positionIndex) {
            var result = 0.0f;
            if (positionIndex < positions.Count - 1) {
                var p1 = positions[positionIndex];
                var p2 = positions[positionIndex + 1];

                var v1 = p1.speedMultiplier * p1.speedDecelerationMultiplier;
                var v2 = p2.speedMultiplier * p2.speedDecelerationMultiplier;
                var s = (float)MMath.GetDistance(p1.point, p2.point);

                var t = 2 * s / (v1 + v2);
                result = (v2 - v1) / t;
            }

            return result;
        }

        public float GetCurrentAcceleration() {
            return GetAcceleration(currentPos);
        }

        public float GetLastMoveDistance() {
            return lastMoveDistance;
        }

        public RobotPosition GetCurrentPosition() {
            return currentRobotPosition;
        }

        public RobotPosition Move(float time) {
            lastMoveDistance = 0.0f;
            var distance = speed * time;
            while (distance > 0 && currentPos < positions.Count - 1) {
                var length = (float)MMath.GetDistance(currentRobotPosition.point, positions[currentPos + 1].point);
                if (distance * currentRobotPosition.speedMultiplier * currentRobotPosition.speedDecelerationMultiplier >= length) {
                    lastMoveDistance += length / (currentRobotPosition.speedMultiplier * currentRobotPosition.speedDecelerationMultiplier);

                    distance -= length / (currentRobotPosition.speedMultiplier * currentRobotPosition.speedDecelerationMultiplier);
                    ++currentPos;
                    currentRobotPosition = positions[currentPos];
                }
                else {
                    lastMoveDistance += distance * currentRobotPosition.speedMultiplier;

                    // TODO fix direction and surface point;
                    var k = distance * currentRobotPosition.speedMultiplier * currentRobotPosition.speedDecelerationMultiplier / length;
                    var newPoint = currentRobotPosition.point + k * (positions[currentPos + 1].point - currentRobotPosition.point);
                    var newDirection = currentRobotPosition.direction * (1 - k) + positions[currentPos + 1].direction * k;
                    currentRobotPosition = new RobotPosition(
                        newPoint,
                        newDirection,
                        positions[currentPos].position,
                        positions[currentPos].speedMultiplier,
                        positions[currentPos].speedDecelerationMultiplier
                    );
                    distance = 0;
                }
            }

            return currentRobotPosition;
        }

        public List<RobotPosition> GetRobotPositions() {
            return positions;
        }

        public void Reset() {
            currentPos = 0;
            currentRobotPosition = positions[currentPos];
        }

        public bool IsFinished() {
            return currentPos == positions.Count - 1;
        }

        public RobotPath GetRobotPathData() {
            var pathItems = new List<RobotPathItem>();
            var i = 0;
            foreach (var pos in GetRobotPositions()) {
                pathItems.Add(new RobotPathItem(pos.position, GetAcceleration(i), pos.speedMultiplier * pos.speedDecelerationMultiplier * speed));
                ++i;
            }

            return new RobotPath(pathItems, speed);
        }
    }

    public class RobotPathBuilder {
        private float maxAcceleration = 0.0f;
        private float minAcceleration = 0.0f;
        private float maxSpeed = 0.0f;

        private IEnumerable<List<Position>> SplitPositions(List<Position> mergedPositions) {
            for (var i = 0; i < mergedPositions.Count; ++i) {
                if (mergedPositions[i].type == Position.PositionType.Start) {
                    var from = i;
                    var to = i + 1;
                    while (mergedPositions[to].type != Position.PositionType.Finish) {
                        ++to;
                    }

                    yield return mergedPositions.GetRange(from, to - from + 1);
                }
            }
        }

        public List<RobotPathProcessor> Build(List<Position> mergedPositions, float speed, float maxSpeed) {
            var result = new List<RobotPathProcessor>();
            foreach (var positions in SplitPositions(mergedPositions)) {
                var robotPositions = new List<RobotPosition>();
                for (var i = 0; i < positions.Count; ++i) {
                    var speedMultiplier = 0.0f;
                    var speedDecelerationMultiplier = 1.0f;
                    if (i >= 0 && i < positions.Count - 1) {
                        if (positions[i].originPoint == positions[i + 1].originPoint) {
                            throw new Exception("Illegal positions");
                        }

                        speedMultiplier = (float)(
                            MMath.GetDistance(positions[i + 1].originPoint, positions[i].originPoint) /
                            MMath.GetDistance(positions[i + 1].surfacePoint, positions[i].surfacePoint)
                        );
                        if (float.IsNaN(speedMultiplier)) {
                            speedMultiplier = 1.0f;
                        }

                        if (!float.IsNaN(maxSpeed) && speedMultiplier * speed > maxSpeed) {
                            speedDecelerationMultiplier = maxSpeed / (speedMultiplier * speed);
                        }
                    }

                    robotPositions.Add(new RobotPosition(
                        positions[i].originPoint,
                        positions[i].paintDirection,
                        positions[i],
                        speedMultiplier,
                        speedDecelerationMultiplier
                    ));
                }

                var robotPath = new RobotPathProcessor(robotPositions);
                robotPath.SetBaseSpeed(speed);
                result.Add(robotPath);

                foreach (var rp in robotPositions) {
                    maxSpeed = Math.Max(rp.speedMultiplier, maxSpeed);
                }

                for (var i = 0; i < robotPositions.Count - 1; ++i) {
                    var v1 = robotPositions[i].speedMultiplier;
                    var v2 = robotPositions[i + 1].speedMultiplier;
                    var s = (float)MMath.GetDistance(robotPositions[i].point, robotPositions[i + 1].point);

                    if (float.IsNaN(s) || s <= 1e-6) {
                        continue;
                    }

                    var t = 2 * s / (v1 + v2);
                    var a = (v2 - v1) / t;

                    minAcceleration = Math.Min(a, minAcceleration);
                    maxAcceleration = Math.Max(a, maxAcceleration);
                }
            }

            return result;
        }
    }
}
