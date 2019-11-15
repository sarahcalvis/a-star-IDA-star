
public class Point {
	String name;
	int x;
	int y;
	int heuristicCost;
	boolean searched;
	public Point(String name, int x, int y, int heuristicCost) {
		this.name = name;
		this.x = x;
		this.y = y;
		this.heuristicCost = heuristicCost;
		searched = false;
	}
	public String toString() {
		return name + " " + heuristicCost;
	}
}
