package Basic;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class Transform {
    public final Point position;
    public final Quaternion orientation;
    public Transform(Point position, Quaternion orientation){
        this.position = position;
        this.orientation = orientation;
    }

    public Vector3D getVector3DPosition() {
        return new Vector3D(position);
    }
}
