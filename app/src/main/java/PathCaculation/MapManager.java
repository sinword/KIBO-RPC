package PathCaculation;

import Basic.Transform;
import Graph.*;

import Basic.Vector3D;

import Basic.Cuboid;
import Basic.LineSegment;
import java.util.*;

public class MapManager {
    public Graph BasicGraph;
    public IMapConfig config;

    public MapManager(IMapConfig config, double distance_from_KOZ) {
        this.config = config;
        createBasicGraph(config.getAllKOZs(), distance_from_KOZ);
    }

    private void createBasicGraph(Cuboid[] KOZs, double distance_from_KOZ) {
        BasicGraph = new Graph(true);
        for (int i = 0; i < KOZs.length; i++) {
            Vector3D[] KOZ_points = KOZs[i].getCuboidOutsideCuboid(distance_from_KOZ).getAvailablePoints();
            for (int j = 0; j < KOZ_points.length; j++) {
                if(!InKIZ(KOZ_points[j] || !NotInKOZ(KOZ_points[j]))){
                    continue;
                }
                Node n = new Node("KOZ" + i + "_point" + j, KOZ_points[j]);
                BasicGraph.addNode(n);
            }
        }

        Node[] AllNodes = BasicGraph.nodes.toArray(new Node[BasicGraph.nodes.size()]);

        for (int i = 0; i < AllNodes.length; i++) {
            for (int j = 0; j < AllNodes.length; j++) {
                if (i == j) {
                    continue;
                }
                LineSegment line = new LineSegment(AllNodes[i].data, AllNodes[j].data);
                if (NotInKOZ(line)) {
                    BasicGraph.addEdge(new Edge(AllNodes[i], AllNodes[j], line.getLength()));
                }
            }
        }
    }



    private boolean NotInKOZ(LineSegment line){
        Cuboid[] KOZs = config.getAllKOZs();
        for (int i = 0; i < KOZs.length; i++) {
            if (KOZs[i].isCross(line)) {
                return false;
            }
        }
        return true;
    }
    private boolean NotInKOZ(Vector3D point){
        Cuboid[] KOZs = config.getAllKOZs();
        for (int i = 0; i < KOZs.length; i++) {
            if (KOZs[i].isInside(point)) {
                return false;
            }
        }
        return true;
    }
    private boolean InKIZ(Vector3D point){
        Cuboid[] KIZs = config.getAllKIZs();
        for (int i = 0; i < KIZs.length; i++){
            if (KIZs[i].isInside(point)){
                return true;
            }
        }
        return false;
    }

    public Integer getShortestAvailablePointID(Vector3D currentPosition, List<Integer> availablePoints, long remainingTime){
        Map<Integer, Double> distanceMap = getAllDistanceFromCurrentPosition(currentPosition, remainingTime);
        double min = Double.MAX_VALUE;
        Integer result = -1;
        for (Map.Entry<Integer, Double> entry : distanceMap.entrySet()){
            if (availablePoints.contains(entry.getKey()) && entry.getValue() < min){
                min = entry.getValue();
                result = entry.getKey();
            }
        }
        return result;
    }

    public Map<Integer, Double> getAllDistanceFromCurrentPosition(Vector3D currentPosition, long remainingTime){
        Map<Integer, Transform> map = config.getTransformMap();
        Map<Integer, Double> result = new HashMap<Integer, Double>();
        for (Map.Entry<Integer, Transform> entry : map.entrySet()){
            Vector3D[] path = getShortestPath(currentPosition, entry.getValue().getVector3DPosition());
            Double distance = getPathLength(path);
            if (entry.getKey() == 8) {
                double estimatedTime = distance / 0.45;
                if (estimatedTime > remainingTime) {
                    distance = Double.MAX_VALUE;
                }
                else {
                    distance = 0.0;
                }
            }
            result.put(entry.getKey(), distance);
        }
        return result;
    }

    private double getPathLength(Vector3D[] path){
        double result = 0;
        for (int i = 0; i < path.length - 1; i++){
            result += path[i].distance(path[i + 1]);
        }
        return result;
    }

    public Vector3D[] getShortestPath(Vector3D from, Vector3D to){
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
        for (int i = 0; i < path.length; i++) {
            result[i] = path[i].data;
        }
        return result;
    }

    private void addNewNode(Graph graph, Node n) {
        graph.addNode(n);
        Node[] AllNodes = graph.nodes.toArray(new Node[graph.nodes.size()]);
        for (int i = 0; i < AllNodes.length; i++) {
            if (AllNodes[i] == n) {
                continue;
            }
            LineSegment line = new LineSegment(AllNodes[i].data, n.data);
            if (NotInKOZ(line)) {
                graph.addEdge(new Edge(AllNodes[i], n, line.getLength()));
            }
        }

    }
}
