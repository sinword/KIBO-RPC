package Graph;

import java.util.Objects;

public class Edge {
    public Node from;
    public Node to;
    public double weight;

    public Edge(Node from, Node to, double weight){
        this.from = from;
        this.to = to;
        this.weight = weight;
    }

    public Edge getReverseEdge(){
        Edge reverseEdge = new Edge(to, from, weight);
        return reverseEdge;
    }
    @Override
    public String toString() {
        return "Edge{" +
                 from.data +
                " -> " + to.data +
                ", weight=" + weight +
                '}';
    }
    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof Edge)) {
            return false;
        }
        Edge other = (Edge) obj;
        return from == other.from && to == other.to && weight == other.weight;
    }
    @Override
    public int hashCode() {
        return Objects.hash(from, to, weight);
    }
}
