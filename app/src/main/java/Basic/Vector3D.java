package Basic;

public class Vector3D {
    final private double x;
    final private double y;
    final private double z;
    public Vector3D(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getZ(){
        return z;
    }

    public double distance(Vector3D other){
        return Math.sqrt(Math.pow(x - other.x, 2) + Math.pow(y - other.y, 2) + Math.pow(z - other.z, 2));
    }
    public double dotProduct(Vector3D other){
        return x * other.x + y * other.y + z * other.z;
    }
    public Vector3D crossProduct(Vector3D other){
        return new Vector3D(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
    }

    public Vector3D normalize(){
        double length = Math.sqrt(x * x + y * y + z * z);
        return new Vector3D(x / length, y / length, z / length);
    }
    public double getNorm(){
        return Math.sqrt(x * x + y * y + z * z);
    }

    public Vector3D scalarMultiply(double scalar){
        return new Vector3D(x * scalar, y * scalar, z * scalar);
    }

    public static Vector3D ZERO = new Vector3D(0, 0, 0);


    @Override
    public String toString(){
        return String.format("(%f, %f, %f)", x, y, z);
    }
    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof Vector3D)) {
            return false;
        }
        Vector3D other = (Vector3D) obj;
        return x == other.x && y == other.y && z == other.z;
    }

    public Vector3D add(Vector3D other){
        return new Vector3D(x + other.x, y + other.y, z + other.z);
    }
    public Vector3D sub(Vector3D other){
        return new Vector3D(x - other.x, y - other.y, z - other.z);
    }
    
}
