
public class Node {
	String name;
	String path;
	int cost;
	public Node(String name, String path, int cost) {
		this.name = name;
		this.path = path;
		this.cost = cost;
	}
	
	public boolean isSameNode(Node n) {
		return this.name.equals(n.name); 
	}

}
