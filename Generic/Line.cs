namespace Library.Generic
{
    public class Line {
        public readonly Point p1;
        public readonly Point p2;

        // (x - x_0) / p0_x = (y - y_0) / p0_y = (z - z_0) / p0_z;
        public readonly float p0X;
        public readonly float p0Y;
        public readonly float p0Z;
        public readonly float x0;
        public readonly float y0;
        public readonly float z0;
        public Line(Point aP1, Point aP2) {
            p1 = aP1;
            p2 = aP2;

            var direction = p1 - p2;

            p0X = direction.x;
            p0Y = direction.y;
            p0Z = direction.z;

            x0 = p1.x;
            y0 = p1.y;
            z0 = p1.z;
        }

        public override int GetHashCode() => new Point(x0, y0, z0).GetHashCode() + new Point(p0X, p0Y, p0Z).GetHashCode();
    }
}
