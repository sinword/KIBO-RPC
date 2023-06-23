package jp.jaxa.iss.kibo.rpc.defaultapk.PathCaculation;

import jp.jaxa.iss.kibo.rpc.defaultapk.Graph.*;

import jp.jaxa.iss.kibo.rpc.defaultapk.Basic.Vector3D;

import jp.jaxa.iss.kibo.rpc.defaultapk.Basic.Cuboid;
import jp.jaxa.iss.kibo.rpc.defaultapk.Basic.LineSegment;
import jp.jaxa.iss.kibo.rpc.defaultapk.Kibo.Point;

public class MapManager{
    public Graph BasicGraph;
    public IMapConfig config;

    public MapManager(IMapConfig config, double distance_from_KOZ) {
        this.config = config;
        createBasicGraph(config.getAllKOZs(), distance_from_KOZ);
    }

    private void createBasicGraph(Cuboid[] KOZs, double distance_from_KOZ) {
        BasicGraph = new Graph(true);
        for (int i = 0; i < KOZs.length; i++) {
            Vector3D[] KOZ_points = KOZs[i].getPointsOutsideCuboid(distance_from_KOZ);
            for (int j = 0; j < KOZ_points.length; j++) {
                Node n = new Node("KOZ" + i + "_point" + j, KOZ_points[j]);
                BasicGraph.addNode(n);
            }
        }

        Node[] AllNodes = BasicGraph.nodes.toArray(new Node[BasicGraph.nodes.size()]);


        for (int i = 0; i < AllNodes.length; i++){
            for (int j = 0; j < AllNodes.length; j++){
                if (i == j){
                    continue;
                }
                LineSegment line = new LineSegment(AllNodes[i].data, AllNodes[j].data);
                if (NotInKOZ(line)){
                    BasicGraph.addEdge(new Edge(AllNodes[i], AllNodes[j], line.getLength()));
                }
            }
        }
    }

    public boolean NotInKOZ(LineSegment line){
        var KOZs = config.getAllKOZs();
        for (int i = 0; i < KOZs.length; i++){
            if (KOZs[i].isCross(line)){
                return false;
            }
        }
        return true;
    }

    public boolean NotInKOZ(Vector3D point){
        var KOZs = config.getAllKOZs();
        for (int i = 0; i < KOZs.length; i++){
            if (KOZs[i].isInside(point)){
                return false;
            }
        }
        return true;
    }

    public Point[] getShortestPath(Point from, Point to) {
        Vector3D[] result = getShortestPath(new Vector3D(from.x, from.y, from.z), new Vector3D(to.x, to.y, to.z));
        Point[] result2 = new Point[result.length];
        for (int i = 0; i < result.length; i++){
            result2[i] = new Point(result[i].getX(), result[i].getY(), result[i].getZ());
        }
        return result2;
    }

    public Vector3D[] getShortestPath(Vector3D from, Vector3D to) {
        if (!NotInKOZ(from) || !NotInKOZ(to)){
            throw new IllegalArgumentException("from point or to point is in KOZ");
        }
        Graph TempGraph = BasicGraph.copy();
        Node start = new Node("start", from);
        Node end = new Node("end", to);
        addNewNode(TempGraph, start);
        addNewNode(TempGraph, end);

        Node[] path = TempGraph.getShortestPath(start, end);
        Vector3D[] result = new Vector3D[path.length];
        for (int i = 0; i < path.length; i++){
            result[i] = path[i].data;
        }
        return result;
    }

    private void addNewNode(Graph graph, Node n){
        graph.addNode(n);
        Node[] AllNodes = graph.nodes.toArray(new Node[graph.nodes.size()]);
        for (int i = 0; i < AllNodes.length; i++){
            if (AllNodes[i] == n){
                continue;
            }
            LineSegment line = new LineSegment(AllNodes[i].data, n.data);
            if (NotInKOZ(line)){
                graph.addEdge(new Edge(AllNodes[i], n, line.getLength()));
            }
        }

    }
}