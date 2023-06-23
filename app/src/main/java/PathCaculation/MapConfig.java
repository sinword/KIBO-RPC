package PathCaculation;

import Basic.Cuboid;
import Basic.Vector3D;

public class MapConfig implements IMapConfig {

    public final Vector3D StartPoint = new Vector3D(9.815, -9.806, 4.293);
    public final Vector3D GoalPoint = new Vector3D(11.143, -6.7607, 4.9654);

    public final Vector3D Point1 = new Vector3D(11.2746, -9.92284, 5.2988);
    public final Vector3D Point2 = new Vector3D(10.612, -9.0709, 4.48);
    public final Vector3D Point3 = new Vector3D(10.71, -7.7, 4.48);
    public final Vector3D Point4 = new Vector3D(10.51 , -6.7185, 5.1804);
    public final Vector3D Point5 = new Vector3D(11.114 , -7.9756, 5.3393);
    public final Vector3D Point6 = new Vector3D(11.355 , -8.9929, 4.7818);
    public final Vector3D Point7 = new Vector3D(11.369 , -8.5518, 4.48);

    public final Vector3D KOZ1_min = new Vector3D(10.783, -9.8899, 4.8385);
    public final Vector3D KOZ1_max = new Vector3D(11.071, -9.6929, 5.0665);
    public final Vector3D KOZ2_min = new Vector3D(10.8652, -9.0734, 4.3861);
    public final Vector3D KOZ2_max = new Vector3D(10.9628, -8.7314, 4.6401);
    public final Vector3D KOZ3_min = new Vector3D(10.185, -8.3826, 4.1475);
    public final Vector3D KOZ3_max = new Vector3D(11.665, -8.2826, 4.6725);
    public final Vector3D KOZ4_min = new Vector3D(10.7955, -8.0635, 5.1055);
    public final Vector3D KOZ4_max = new Vector3D(11.3525, -7.7305, 5.1305);
    public final Vector3D KOZ5_min = new Vector3D(10.563, -7.1449, 4.6544);
    public final Vector3D KOZ5_max = new Vector3D(10.709, -6.8099, 4.8164);
    public final Vector3D KIZ1_min = new Vector3D(10.3, -10.2, 4.32);
    public final Vector3D KIZ1_max = new Vector3D(11.55, -6.0, 5.57);
    public final Vector3D KIZ2_min = new Vector3D(9.5, -10.5, 4.02);
    public final Vector3D KIZ2_max = new Vector3D(10.5, -9.6, 4.8);

    public final Cuboid KOZ1 = new Cuboid(KOZ1_max, KOZ1_min);
    public final Cuboid KOZ2 = new Cuboid(KOZ2_max, KOZ2_min);
    public final Cuboid KOZ3 = new Cuboid(KOZ3_max, KOZ3_min);
    public final Cuboid KOZ4 = new Cuboid(KOZ4_max, KOZ4_min);
    public final Cuboid KOZ5 = new Cuboid(KOZ5_max, KOZ5_min);
    public final Cuboid KIZ1 = new Cuboid(KIZ1_max, KIZ1_min);
    public final Cuboid KIZ2 = new Cuboid(KIZ2_max, KIZ2_min);

    public final Vector3D[] AllPoints = new Vector3D[]{Point1, Point2, Point3, Point4, Point5, Point6, Point7};
    public final Cuboid[] AllKOZs = new Cuboid[]{KOZ1, KOZ2, KOZ3, KOZ4, KOZ5};
    public final Cuboid[] AllKIZs = new Cuboid[]{KIZ1, KIZ2};
    @Override
    public Cuboid[] getAllKOZs() {
        return AllKOZs;
    }
}
