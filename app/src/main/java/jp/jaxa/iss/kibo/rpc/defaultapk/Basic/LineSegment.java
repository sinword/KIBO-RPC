package jp.jaxa.iss.kibo.rpc.defaultapk.Basic;

public class LineSegment {
    public final Vector3D start;
    public final Vector3D end;

    public LineSegment(Vector3D start, Vector3D end) {
        this.start = start;
        this.end = end;
    }
    public double getLength(){
        return start.distance(end);
    }
    public Vector3D getDirection(){
        return new Vector3D(end.getX() - start.getX(), end.getY() - start.getY(), end.getZ() - start.getZ());
    }

}
