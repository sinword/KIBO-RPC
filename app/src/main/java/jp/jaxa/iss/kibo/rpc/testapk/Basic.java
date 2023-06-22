package jp.jaxa.iss.kibo.rpc.defaultapk;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import java.util.*;


public class LineSegment {
    public final Point start;
    public final Point end;

    public LineSegment(Point start, Point end) {
        this.start = start;
        this.end = end;
    }
    public double getLength(){
        return Math.sqrt(Math.pow(start.x - end.x, 2) + Math.pow(start.y - end.y, 2) + Math.pow(start.z - end.z, 2));
    }
}

public class Cuboid {
    public final Point maxPoint;
    public final Point minPoint;

    public Cuboid(Point maxPoint, Point minPoint) {
        this.maxPoint = maxPoint;
        this.minPoint = minPoint;
    }

    public boolean isInside(Point point) {
        if (point.x > minPoint.x && point.x < maxPoint.x && point.y > minPoint.y && point.y < maxPoint.y
                && point.z > minPoint.z && point.z < maxPoint.z) {
            return true;
        }
        return false;
    }

    public boolean isInside(LineSegment line) {
        Point lineStart = line.start;
        Point lineEnd = line.end;

        LineSegment[] cuboidEdges = getCuboidEdges();
        for (LineSegment cuboidEdge : cuboidEdges) {
            // 檢查線段是否與 Cuboid 的邊界線段相交
            if (lineSegmentIntersects(cuboidEdge, lineStart, lineEnd)) {
                return true;
            }
        }

        return false;
    }

    private boolean lineSegmentIntersects(LineSegment edge, Point lineStart, Point lineEnd) {
        // 使用三維線段交叉判斷法檢查兩線段是否相交
        double dx1 = edge.end.x - edge.start.x;
        double dy1 = edge.end.y - edge.start.y;
        double dz1 = edge.end.z - edge.start.z;

        double dx2 = lineEnd.x - lineStart.x;
        double dy2 = lineEnd.y - lineStart.y;
        double dz2 = lineEnd.z - lineStart.z;

        double cross1 = dy1 * dz2 - dz1 * dy2;
        double cross2 = dz1 * dx2 - dx1 * dz2;
        double cross3 = dx1 * dy2 - dy1 * dx2;

        if (cross1 == 0 && cross2 == 0 && cross3 == 0) {
            // 兩線段共線
            return collinearSegmentsIntersect(edge, lineStart, lineEnd);
        }

        double t1 = ((lineStart.x - edge.start.x) * dy2 - (lineStart.y - edge.start.y) * dx2) / cross3;
        double t2 = ((lineStart.x - edge.start.x) * dy1 - (lineStart.y - edge.start.y) * dx1) / cross3;
        double t3 = ((edge.start.x - lineStart.x) * dz2 - (edge.start.z - lineStart.z) * dx2) / cross2;
        double t4 = ((edge.start.x - lineStart.x) * dz1 - (edge.start.z - lineStart.z) * dx1) / cross2;

        // 檢查兩條線段是否相交
        if (t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1 && t3 >= 0 && t3 <= 1 && t4 >= 0 && t4 <= 1) {
            return true;
        }

        return false;
    }

    private boolean collinearSegmentsIntersect(LineSegment edge, Point lineStart, Point lineEnd) {
        // 檢查兩共線線段是否重疊
        double epsilon = 1e-10;

        // 計算線段的參數值
        double edgeTStart = 0;
        double edgeTEnd = 1;
        double lineTStart = ((lineStart.x - edge.start.x) * (edge.end.x - edge.start.x)
                + (lineStart.y - edge.start.y) * (edge.end.y - edge.start.y)
                + (lineStart.z - edge.start.z) * (edge.end.z - edge.start.z))
                / edge.getLengthSquared();
        double lineTEnd = ((lineEnd.x - edge.start.x) * (edge.end.x - edge.start.x)
                + (lineEnd.y - edge.start.y) * (edge.end.y - edge.start.y)
                + (lineEnd.z - edge.start.z) * (edge.end.z - edge.start.z))
                / edge.getLengthSquared();

        // 檢查參數值範圍是否有重疊
        if (Math.abs(edgeTStart - edgeTEnd) < epsilon && Math.abs(lineTStart - lineTEnd) < epsilon) {
            // 兩個線段都是點，比較起始點是否相同
            return Math.abs(edgeTStart - lineTStart) < epsilon;
        }

        return false;
    }

    public Point[] getCuboidPoints(){
        // 取得Cuboid的所有頂點
        Point[] points = new Point[8];

        points[0] = minPoint;
        points[1] = new Point(minPoint.x, maxPoint.y, minPoint.z);
        points[2] = new Point(maxPoint.x, maxPoint.y, minPoint.z);
        points[3] = new Point(maxPoint.x, minPoint.y, minPoint.z);
        points[4] = new Point(minPoint.x, minPoint.y, maxPoint.z);
        points[5] = new Point(minPoint.x, maxPoint.y, maxPoint.z);
        points[6] = maxPoint;
        points[7] = new Point(maxPoint.x, minPoint.y, maxPoint.z);

        return points;
    }

    public LineSegment[] getCuboidEdges() {
        // 取得Cuboid的所有邊界線段
        LineSegment[] edges = new LineSegment[12];

        Point[] points = getCuboidPoints();

        edges[0] = new LineSegment(points[0], points[1]);
        edges[1] = new LineSegment(points[1], points[2]);
        edges[2] = new LineSegment(points[2], points[3]);
        edges[3] = new LineSegment(points[3], points[0]);
        edges[4] = new LineSegment(points[0], points[4]);
        edges[5] = new LineSegment(points[1], points[5]);
        edges[6] = new LineSegment(points[2], points[6]);
        edges[7] = new LineSegment(points[3], points[7]);
        edges[8] = new LineSegment(points[4], points[5]);
        edges[9] = new LineSegment(points[5], points[6]);
        edges[10] = new LineSegment(points[6], points[7]);
        edges[11] = new LineSegment(points[7], points[4]);


        return edges;
    }
}
