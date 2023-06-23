package jp.jaxa.iss.kibo.rpc.defaultapk.PathCaculation;

import jp.jaxa.iss.kibo.rpc.defaultapk.Basic.Cuboid;
import jp.jaxa.iss.kibo.rpc.defaultapk.Kibo.Point;

public class MapConfig implements IMapConfig {

    public final Point StartPoint = new Point(9.815, -9.806, 4.293);
    public final Point GoalPoint = new Point(11.143, -6.7607, 4.9654);

    public final Point Point1 = new Point(11.2746, -9.92284, 5.2988);
    public final Point Point2 = new Point(10.612, -9.0709, 4.48);
    public final Point Point3 = new Point(10.71, -7.7, 4.48);
    public final Point Point4 = new Point(10.51 , -6.7185, 5.1804);
    public final Point Point5 = new Point(11.114 , -7.9756, 5.3393);
    public final Point Point6 = new Point(11.355 , -8.9929, 4.7818);
    public final Point Point7 = new Point(11.369 , -8.5518, 4.48);

    public final Point KOZ1_min = new Point(10.783, -9.8899, 4.8385);
    public final Point KOZ1_max = new Point(11.071, -9.6929, 5.0665);
    public final Point KOZ2_min = new Point(10.8652, -9.0734, 4.3861);
    public final Point KOZ2_max = new Point(10.9628, -8.7314, 4.6401);
    public final Point KOZ3_min = new Point(10.185, -8.3826, 4.1475);
    public final Point KOZ3_max = new Point(11.665, -8.2826, 4.6725);
    public final Point KOZ4_min = new Point(10.7955, -8.0635, 5.1055);
    public final Point KOZ4_max = new Point(11.3525, -7.7305, 5.1305);
    public final Point KOZ5_min = new Point(10.563, -7.1449, 4.6544);
    public final Point KOZ5_max = new Point(10.709, -6.8099, 4.8164);
    public final Point KIZ1_min = new Point(10.3, -10.2, 4.32);
    public final Point KIZ1_max = new Point(11.55, -6.0, 5.57);
    public final Point KIZ2_min = new Point(9.5, -10.5, 4.02);
    public final Point KIZ2_max = new Point(10.5, -9.6, 4.8);

    public final Cuboid KOZ1 = new Cuboid(KOZ1_max, KOZ1_min);
    public final Cuboid KOZ2 = new Cuboid(KOZ2_max, KOZ2_min);
    public final Cuboid KOZ3 = new Cuboid(KOZ3_max, KOZ3_min);
    public final Cuboid KOZ4 = new Cuboid(KOZ4_max, KOZ4_min);
    public final Cuboid KOZ5 = new Cuboid(KOZ5_max, KOZ5_min);
    public final Cuboid KIZ1 = new Cuboid(KIZ1_max, KIZ1_min);
    public final Cuboid KIZ2 = new Cuboid(KIZ2_max, KIZ2_min);

    public final Point[] AllPoints = new Point[]{Point1, Point2, Point3, Point4, Point5, Point6, Point7};
    public final Cuboid[] AllKOZs = new Cuboid[]{KOZ1, KOZ2, KOZ3, KOZ4, KOZ5};
    public final Cuboid[] AllKIZs = new Cuboid[]{KIZ1, KIZ2};
    @Override
    public Cuboid[] getAllKOZs() {
        return AllKOZs;
    }
}
