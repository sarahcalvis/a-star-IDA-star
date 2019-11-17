////////////////////////////////////
// A simple point on an x-y plane //
// We use these to generate nodes //
////////////////////////////////////
public class Point {
	String name;
	int x;
	int y;
	int heuristicCost;
	boolean searched;
	
	/**
	 * @param name: Point name
	 * @param x: Point x coordinate
	 * @param y: Point y coordinate
	 * @param heuristicCost: Estimated cost from point to goal
	 */
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
