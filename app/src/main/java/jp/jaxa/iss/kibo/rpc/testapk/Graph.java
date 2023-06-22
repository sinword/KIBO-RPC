import java.util.*;

public class Graph{
    public Set<Node> nodes;
    public Set<Edge> edges;

    public Graph(){
        nodes = new HashSet<Node>();
        edges = new HashSet<Edge>();
    }
    public Graph(Set<Node> nodes, Set<Edge> edges){
        this.nodes = nodes;
        this.edges = edges;
    }
    public void addNode(Node node){
        nodes.add(node);
    }
    public void addEdge(Edge edge){
        edges.add(edge);
        
        // 添加反向邊
        edges.add(edge.GetReverseEdge());
    }

    public List<Node<T>> getNeighbors(Node node){
        List<Node> neighbors = new ArrayList<Node>();
        for(Edge edge : edges){
            if(edge.from == node){
                neighbors.add(edge.to);
            }
        }
        return neighbors;
    }
    public T2 getWeight(Node from, Node to){
        for(Edge edge : edges){
            if(edge.from == from && edge.to == to){
                return edge.weight;
            }
        }
        return null;
    }

}
public class Node{
    public String name;
    public Point data;
}
public class Edge{
    public Node from;
    public Node to;
    public double weight;

    public GetReverseEdge(){
        Edge reverseEdge = new Edge();
        reverseEdge.from = to;
        reverseEdge.to = from;
        reverseEdge.weight = weight;
        return reverseEdge;
    }
}