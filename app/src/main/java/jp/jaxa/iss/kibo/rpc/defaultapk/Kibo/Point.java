package jp.jaxa.iss.kibo.rpc.defaultapk.Kibo;


public class Point {
    final public double x;
    final public double y;
    final public double z;
    public Point(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }
    @Override
    public String toString(){
        return String.format("(%f, %f, %f)", x, y, z);
    }
    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof Point)) {
            return false;
        }
        Point other = (Point) obj;
        return x == other.x && y == other.y && z == other.z;
    }
}
