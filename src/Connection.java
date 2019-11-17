////////////////////////////////////////////////
// Represents a connection between two points //
////////////////////////////////////////////////
public class Connection {
	Point a;
	Point b;
	int distance;
	
	/**
	 * Initialize the points
	 * @param a: The parent point
	 * @param b: The child point
	 */
	public Connection(Point a, Point b) {
		this.a = a;
		this.b = b;
		int x = a.x - b.x;
		int y = a.y - b.y;
		this.distance = (int)Math.sqrt((double)x*x + (double)y*y);
	}
	public String toString() {
		return a.name + " " + b.name + " " + distance;
	}
}
