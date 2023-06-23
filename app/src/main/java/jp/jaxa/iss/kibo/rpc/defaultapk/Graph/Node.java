package jp.jaxa.iss.kibo.rpc.defaultapk.Graph;
import jp.jaxa.iss.kibo.rpc.defaultapk.Basic.Vector3D;
import java.util.Objects;


public class Node {
    public String name;
    public Vector3D data;

    public Node(String name, Vector3D data){
        this.name = name;
        this.data = data;
    }


    @Override
    public String toString(){
        return name + " " + data.toString();
    }
    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        Node other = (Node) obj;
        return Objects.equals(data, other.data) && Objects.equals(name, other.name);
    }
    @Override
    public int hashCode() {
        return Objects.hash(data, name);
    }
}
