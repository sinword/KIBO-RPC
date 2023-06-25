package Basic;

public class Cuboid {
    public final Vector3D maxPoint;
    public final Vector3D minPoint;

    public Cuboid(Vector3D maxPoint, Vector3D minPoint) {
        this.maxPoint = maxPoint;
        this.minPoint = minPoint;

    }

    public boolean isInside(Vector3D point) {
        // 判斷點是否在Cuboid內
        return point.getX() <= maxPoint.getX() && point.getX() >= minPoint.getX() && point.getY() <= maxPoint.getY()
                && point.getY() >= minPoint.getY() && point.getZ() <= maxPoint.getZ()
                && point.getZ() >= minPoint.getZ();

    }

    public boolean isCross(LineSegment line) {
        if (isInside(line.start) || isInside(line.end)) {
            return true;
        }
        Plane[] planes = getAllPlanes();
        for (Plane plane : planes) {
            if (plane.hasIntersection(line)) {
                return true;
            }
        }
        return false;
    }

    public Vector3D[] getCuboidPoints() {
        // 取得Cuboid的所有頂點
        Vector3D[] points = new Vector3D[8];

        points[0] = minPoint;
        points[1] = new Vector3D(minPoint.getX(), maxPoint.getY(), minPoint.getZ());
        points[2] = new Vector3D(maxPoint.getX(), maxPoint.getY(), minPoint.getZ());
        points[3] = new Vector3D(maxPoint.getX(), minPoint.getY(), minPoint.getZ());
        points[4] = new Vector3D(minPoint.getX(), minPoint.getY(), maxPoint.getZ());
        points[5] = new Vector3D(minPoint.getX(), maxPoint.getY(), maxPoint.getZ());
        points[6] = maxPoint;
        points[7] = new Vector3D(maxPoint.getX(), minPoint.getY(), maxPoint.getZ());

        return points;
    }

    public Vector3D[] getPointsOutsideCuboid(double distance) {
        Vector3D newMax = new Vector3D(maxPoint.getX() + distance, maxPoint.getY() + distance,
                maxPoint.getZ() + distance);
        Vector3D newMin = new Vector3D(minPoint.getX() - distance, minPoint.getY() - distance,
                minPoint.getZ() - distance);
        Cuboid newCuboid = new Cuboid(newMax, newMin);
        return newCuboid.getCuboidPoints();
    }

    public Plane[] getAllPlanes() {
        // 取得Cuboid的所有平面
        Plane[] planes = new Plane[6];
        Vector3D[] ps = getCuboidPoints();

        planes[0] = new Plane(ps[0], ps[1].sub(ps[0]), ps[3].sub(ps[0])); // top
        planes[1] = new Plane(ps[1], ps[5].sub(ps[1]), ps[2].sub(ps[1])); // front
        planes[2] = new Plane(ps[0], ps[4].sub(ps[0]), ps[1].sub(ps[0])); // left
        planes[3] = new Plane(ps[3], ps[2].sub(ps[3]), ps[7].sub(ps[3])); // right
        planes[4] = new Plane(ps[0], ps[3].sub(ps[0]), ps[4].sub(ps[0])); // back
        planes[5] = new Plane(ps[4], ps[5].sub(ps[4]), ps[7].sub(ps[4])); // bottom

        return planes;
    }

}
