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
	 * @param numCities: number of cities for which we will generate a graph
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
	 * Generate numCities random points
	 */
	public void generateCities() {
		p.add(new Point("0", 0, 0, (int)Math.sqrt(2 * (double) spaceSize * spaceSize)));	// Starting node
		p.add(new Point("1", numCities*10, numCities*10, 0));								// Goal node
		for (int i = 2; i < numCities; i++) {
			int x = r.nextInt(spaceSize);													// Random x coordinate
			int y = r.nextInt(spaceSize);													// Random y coordinate
			int heuristicCost = (int)Math.floor(Math.sqrt((double)x*x + y*y));				// Use Pythagorean theorem
			p.add(new Point(Integer.toString(i), x, y, heuristicCost));
		}		
	}
	
	/**
	 * Generate connections using random numbers until the graph is fully connected
	 */
	public void generateConnections() {
		while(!graphIsConnected()) {
			for (int i = 0; i < initialConnections; i++) {
				int pa = r.nextInt(numCities);
				int pb = r.nextInt(numCities);
				if (!areConnected(pa, pb)) c.add(new Connection(p.get(pa), p.get(pb)));
			}
		}
	}
	/**
	 * Iterates over the connections to find if there is one between the current point
	 * also returns true if the two points are the same because we do not want loops in our graph
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
	 * Iterates over the connections to get all the points connected to the input point
	 * @param point a point whose neighbors we desire to find
	 * @return a list of the point's neighbors.
	 */
	public ArrayList<Point> getNeighbors(Point point) {
		ArrayList<Point> neighbors = new ArrayList<Point>();
		for (Connection conn: c) if (conn.a.name.equals(point.name) || conn.b.name.equals(point.name)) neighbors.add(conn.b);
		return neighbors;
	}
	/**
	 * Marks all the points as unsearched
	 * Calls the search function on the starting point.
	 * If any of the points are unsearched after the recursive search function returns,
	 * the graph is unconnected.
	 * @return whether the graph is connected
	 */
	public boolean graphIsConnected() {
		for (Point point: p) point.searched = false;
		search(p.get(0));
		for (Point point: p) if (point.searched == false) return false;
		return true;
	}
	
	/**
	 * Recursively iterates over all the points connected to point and marks them as searched.
	 * @param point: the starting point
	 */
	public void search(Point point) {
		point.searched = true;
		for (Point neighbor: getNeighbors(point)) if(neighbor.searched == false) search(neighbor);
	}
	
	/**
	 * Write the points and connections to a file
	 */
	public void writeToFile() {
		// First write them to a string
		String points = "";
		String connections = "";
		for (Point point: p) points += point.toString() + "\n";
		for (Connection connection: c) connections += connection.toString() + "\n";
		
		// Then write them to a file
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
		catch (IOException e) {
			System.out.println(e.getMessage());
		}
	}
}
