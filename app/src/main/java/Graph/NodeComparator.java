package Graph;

import java.util.Comparator;
import java.util.Map;

public class NodeComparator implements Comparator<Node> {
    private Map<Node, Double> distances;

    public NodeComparator(Map<Node, Double> distances) {
        this.distances = distances;
    }

    @Override
    public int compare(Node node1, Node node2) {
        double distance1 = distances.get(node1);
        double distance2 = distances.get(node2);

        return Double.compare(distance1, distance2);
    }
}
