/////////////////////////////////////////////////////////////
// Represents a node in a connected graph we are searching //
/////////////////////////////////////////////////////////////
public class Node {
	String name;
	String path;
	int cost;
	
	/**
	 * @param name: Name of node
	 * @param path: Path to get to node
	 * @param cost: Cost to get to node + estimated cost to the goal from this node
	 */
	public Node(String name, String path, int cost) {
		this.name = name;
		this.path = path;
		this.cost = cost;
	}
	
	public String toString() {
		return "{" + name + ", [" + path + "], " + cost + "}";
	}

}
