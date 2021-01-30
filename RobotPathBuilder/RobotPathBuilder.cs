using System;
using System.Collections.Generic;
using Library.Generic;

namespace Library.RobotPathBuilder {
    public class RobotPosition {
        public readonly Point point;
        public readonly Point direction;
        public readonly float speedMultiplier;
        public readonly Position position;

        public RobotPosition(Point aPoint, Point aDirection, Position aPosition, float aSpeedMultiplier) {
            point = aPoint;
            direction = aDirection;
            position = aPosition;
            speedMultiplier = aSpeedMultiplier;
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
                result = speed * currentRobotPosition.speedMultiplier;
            }

            return result;
        }

        public float GetAcceleration(int positionIndex) {
            var result = 0.0f;
            if (positionIndex < positions.Count - 1) {
                var p1 = positions[positionIndex];
                var p2 = positions[positionIndex + 1];

                var v1 = p1.speedMultiplier;
                var v2 = p2.speedMultiplier;
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
                if (distance * currentRobotPosition.speedMultiplier >= length) {
                    lastMoveDistance += length / currentRobotPosition.speedMultiplier;

                    distance -= length / currentRobotPosition.speedMultiplier;
                    ++currentPos;
                    currentRobotPosition = positions[currentPos];
                }
                else {
                    lastMoveDistance += distance * currentRobotPosition.speedMultiplier;

                    var k = distance * currentRobotPosition.speedMultiplier / length;
                    var newPoint = currentRobotPosition.point + k * (positions[currentPos + 1].point - currentRobotPosition.point);
                    currentRobotPosition = new RobotPosition(newPoint, positions[currentPos].direction, positions[currentPos].position, positions[currentPos].speedMultiplier);
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

        public RobotPath GetRobotPathData(float paintSpeed) {
            var pathItems = new List<RobotPathItem>();
            var i = 0;
            foreach (var pos in GetRobotPositions()) {
                pathItems.Add(new RobotPathItem(pos.position, GetAcceleration(i), pos.speedMultiplier * paintSpeed));
                ++i;
            }

            return new RobotPath(pathItems, paintSpeed);
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

        public List<RobotPathProcessor> Build(List<Position> mergedPositions, float speed) {
            var result = new List<RobotPathProcessor>();
            foreach (var positions in SplitPositions(mergedPositions)) {
                var robotPositions = new List<RobotPosition>();
                for (var i = 0; i < positions.Count; ++i) {
                    var speedMultiplier = 1.0f;
                    if (i < positions.Count - 1) {
                        if (positions[i].originPoint == positions[i + 1].originPoint) {
                            throw new Exception("Illegal positions");
                        }

                        speedMultiplier = (float)(
                            MMath.GetDistance(positions[i + 1].originPoint, positions[i].originPoint) /
                            MMath.GetDistance(positions[i + 1].surfacePoint, positions[i].surfacePoint)
                        );
                        if (speedMultiplier is float.NaN) {
                            speedMultiplier = 1.0f;
                        }
                    }
                    robotPositions.Add(new RobotPosition(positions[i].originPoint, positions[i].paintDirection, positions[i], speedMultiplier));
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