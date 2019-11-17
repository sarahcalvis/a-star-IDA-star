/////////////////////////////////
// Generates a connected graph //
/////////////////////////////////

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

public class GraphGenerator {
	public int numCities;
	public int spaceSize;
	public int initialConnections;
	ArrayList<Point> p;
	ArrayList<Connection> c;
	Random r;
	
	/**
	 * @param numCities number of cities for which we will generate a graph
	 */
	public GraphGenerator(int numCities) {
		this.numCities = numCities;
		spaceSize = 10*numCities;
		initialConnections = (int) ((double)numCities * Math.log((double)numCities)/Math.log(2.0));
		p = new ArrayList<Point>();
		c = new ArrayList<Connection>();
		r = new Random();
	}
	
	/**
	 * generate numCities random points
	 */
	public void generateCities() {
		//starting node
		p.add(new Point("0", 0, 0, (int)Math.sqrt(2 * (double) spaceSize * spaceSize)));
		//goal node
		p.add(new Point("1", numCities*10, numCities*10, 0));
		for (int i = 2; i < numCities; i++) {
			int x = r.nextInt(spaceSize);
			int y = r.nextInt(spaceSize);
			int heuristicCost = (int)Math.floor(Math.sqrt((double)x*x + y*y));
			p.add(new Point(Integer.toString(i), x, y, heuristicCost));
		}		

	}
	/**
	 * generate connections until the graph is fully connected
	 */
	public void generateConnections() {
		int timesGen = 0;
		while(!isConnected()) {
			for (int i = 0; i < initialConnections; i++) {
				int pa = r.nextInt(numCities);
				int pb = r.nextInt(numCities);
				if (!areConnected(pa, pb)) c.add(new Connection(p.get(pa), p.get(pb)));
			}
			timesGen++;
		}
	}
	/**
	 * @param a a point
	 * @param b a point
	 * @return whether the two points are connected
	 */
	public boolean areConnected(int a, int b) {
		for (Connection conn: c) {
			if (a == b || (conn.a.name.equals(Integer.toString(a))&&conn.b.name.equals(Integer.toString(b)))) return true;
		}
		return false;
	}
	/**
	 * @param point a point whose neighbors we desire to find
	 * @return a list of the points neighbors.
	 */
	public ArrayList<Point> getNeighbors(Point point) {
		ArrayList<Point> neighbors = new ArrayList<Point>();
		for (Connection conn: c) if (conn.a.name.equals(point.name) || conn.b.name.equals(point.name)) neighbors.add(conn.b);
		return neighbors;
	}
	/**
	 * @return whether the graph is connected
	 */
	public boolean isConnected() {
		for (Point point: p) point.searched = false;
		search(p.get(0));
		for (Point point: p) if (point.searched == false) return false;
		return true;
	}
	/**
	 * @param point
	 * @param map
	 * @return a map with all connected points marked true
	 */
	public void search(Point point) {
		point.searched = true;
		for (Point neighbor: getNeighbors(point)) if(neighbor.searched == false) search(neighbor);
	}
	public void writeToFile() {
		String points = "";
		String connections = "";
		for (Point point: p) points += point.toString() + "\n";
		for (Connection connection: c) connections += connection.toString() + "\n";
		try {
			FileWriter fp = new FileWriter("points.txt", false);
			BufferedWriter outp = new BufferedWriter(fp);
			outp.write(points);
			outp.close();
			FileWriter cf = new FileWriter("connections.txt", false);
			BufferedWriter outc = new BufferedWriter(cf);
			outc.write(connections);
			outc.close();
		}
		catch (IOException e) System.out.println(e.getMessage());
	}
}
