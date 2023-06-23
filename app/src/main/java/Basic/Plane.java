package Basic;

public class Plane {
    /** Offset of the origin with respect to the plane. */

    /** Origin of the plane frame. */
    private Vector3D origin;

    /** First vector of the plane frame (in plane). */
    private Vector3D u;

    /** Second vector of the plane frame (in plane). */
    private Vector3D v;

    /** Third vector of the plane frame (plane normal). */
    private Vector3D w;

    public Plane(Vector3D origin, Vector3D u, Vector3D v) {
        this.origin = origin;
        this.u = u;
        this.v = v;
        this.w = u.crossProduct(v);
    }

    public boolean hasIntersection(final LineSegment line) {
        // 判斷線段是否與平面相交
        Vector3D lineDirection = line.getDirection();
        Vector3D lineStart = line.start;
        Vector3D lineOrigin = lineStart;
        Vector3D lineNormal = lineDirection.crossProduct(w);
        if (lineNormal.equals(Vector3D.ZERO)) {
            // 平面與線段平行
            return false;
        }
        double t = (origin.sub(lineOrigin).dotProduct(w)) / (lineDirection.dotProduct(w));
        if (t < 0 || t > 1) {
            // 線段與平面不相交
            return false;
        }
        Vector3D intersection = lineStart.add(lineDirection.scalarMultiply(t));
        Vector3D intersectionDirection = intersection.sub(origin);
        double x = intersectionDirection.dotProduct(u);
        double y = intersectionDirection.dotProduct(v);
        if (x < 0 || x > u.dotProduct(u) || y < 0 || y > v.dotProduct(v)) {
            // 線段與平面不相交
            return false;
        }
        return true;
    }

}
