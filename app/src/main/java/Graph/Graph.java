package Graph;

import java.util.*;

public class Graph{
    public Set<Node> nodes;
    public Set<Edge> edges;
    public final boolean IsBidirectional;

    public Graph(boolean isBidirectional){
        this(new HashSet<>(), new HashSet<>(), isBidirectional);
    }

    public Graph(Set<Node> nodes, Set<Edge> edges, boolean isBidirectional){
        this.nodes = nodes;
        this.edges = edges;
        IsBidirectional = isBidirectional;
    }

    public Graph copy(){
        Set<Node> newNodes = new HashSet<>();
        Set<Edge> newEdges = new HashSet<>();
        for(Node node : nodes){
            newNodes.add(new Node(node.name ,node.data));
        }
        for(Edge edge : edges){
            newEdges.add(new Edge(edge.from, edge.to, edge.weight));
        }
        return new Graph(newNodes, newEdges, IsBidirectional);
    }

    public void addNode(Node node){
        nodes.add(node);
    }
    public void addEdge(Edge edge){
        edges.add(edge);
        
        if(IsBidirectional)
            edges.add(edge.getReverseEdge());
    }

    public List<Node> getNeighbors(Node node){
        List<Node> neighbors = new ArrayList<Node>();
        for(Edge edge : edges){
            if(edge.from == node){
                neighbors.add(edge.to);
            }
        }
        return neighbors;
    }
    public double getWeight(Node from, Node to){
        for(Edge edge : edges){
            if(edge.from == from && edge.to == to){
                return edge.weight;
            }
        }
        throw new RuntimeException("No such edge.");
    }

    public Node[] getShortestPath(Node start, Node end) {
        if (!nodes.contains(start) || !nodes.contains(end)) {
            throw new RuntimeException("Start or end node is not in the graph.");
        }
        return dijkstra(start, end);
    }
    private Node[] dijkstra(Node start, Node end) {
        Map<Node, Double> distances = new HashMap<>();
        Map<Node, Node> previous = new HashMap<>();
        PriorityQueue<Node> nodes = new PriorityQueue<>(Comparator.comparingDouble(distances::get));
        Set<Node> visited = new HashSet<>();

        for (Node node : this.nodes) {
            distances.put(node, Double.POSITIVE_INFINITY);
            previous.put(node, null);
        }

        distances.put(start, 0.0);
        nodes.add(start);

        while (!nodes.isEmpty()) {
            Node current = nodes.poll();
            visited.add(current);

            if (current == end) {
                break;
            }

            for (Node neighbor : getNeighbors(current)) {
                if (visited.contains(neighbor)) {
                    continue;
                }

                double newDistance = distances.get(current) + getWeight(current, neighbor);
                if (newDistance < distances.get(neighbor)) {
                    distances.put(neighbor, newDistance);
                    previous.put(neighbor, current);
                    nodes.add(neighbor);
                }
            }
        }

        List<Node> path = new ArrayList<>();
        Node current = end;
        while (current != null) {
            path.add(current);
            current = previous.get(current);
        }
        Collections.reverse(path);

        return path.toArray(new Node[path.size()]);
    }
    public void dump_edges(){
        for(Edge edge : edges){
            System.out.println(edge.from.name + " -> " + edge.to.name + " : " + edge.weight);
        }
    }
    public void dump_nodes(){
        for(Node node : nodes){
            System.out.println(node.name + " : " + node.data.toString());
        }
    }
}