///////////////////////////////////
// Represents a neighboring city //
///////////////////////////////////
public class Neighbor {
	String name;
	int cost;
	/**
	 * Initializes a neighbor
	 * @param name: name of neighbor
	 * @param cost: cost from parent to neighbor
	 */
	public Neighbor(String name, int cost) {
		this.name = name;
		this.cost = cost;
	}
}
