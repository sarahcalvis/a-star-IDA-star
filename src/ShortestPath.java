import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Scanner;
import java.util.TreeMap;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.Arrays;
import java.util.LinkedList;
public class ShortestPath {
	/**
	 * @param args
	 * @throws FileNotFoundException
	 * Sets up for A* and D* and then performs these searches
	 */
	public static void main (String[] args) throws FileNotFoundException {
		Node start = new Node("Ar", "Ar", 0);	//starting node
		String goal = "Bu";						//goal node name
		
		Map<String, List<Neighbor>> neighbors = getNeighbors("ActualDistances.txt");		//map of distances between neighboring nodes
		Map<String, Integer> straightLines = getStraightLines("StraightLineDistance.txt");	//map of straight line distances between nodes and goal node

		aStar(start, goal, neighbors, straightLines);	//call aStar
		dStar(start, goal, neighbors, straightLines);	//call dStar

	}

	/**
	 * @param start: The starting node
	 * @param goal: The goal node's name
	 * @param neighbors: A list that holds (node name, neighbor) pairs
	 * @param straightLines: A list of the straight line distances from all the nodes to the goal node
	 * Performs A*, a shortest path algorithm based on the Triangle Inequality
	 */
	public static void aStar(Node start, String goal, Map<String, List<Neighbor>> neighbors, Map<String, Integer> straightLines) {
		int cost = Integer.MAX_VALUE;	// Initialize to infinity because first solution found will be better
		String bestPath = "";			// Store the best path to goal

		NodeComparator compare = new NodeComparator();						// Use to compare nodes
		PriorityQueue<Node> nodeQueue = new PriorityQueue<Node>(compare);	// Holds nodes that must be traversed

		nodeQueue.add(start);			// Add the initial node

		// Iterate through the node queue
		while (!nodeQueue.isEmpty()) {
			Node currNode = nodeQueue.remove();	// Take the smallest node
			
			// If we found the goal
			if (currNode.name.equals(goal)) {	
				// If this path is better than the reigning path
				if (currNode.cost < cost) {	
					cost = currNode.cost;		// Save path cost
					bestPath = currNode.path;	// Save path

					// All the untraversed paths are worse than our solution therefore we found best path
					if (nodeQueue.peek().cost > cost) { break; }
					
				} // End if (currNode.cost < cost)
			} // End if (currNode.name.equals(goal))
			
			// If the goal wasn't found, add the neighbors of the found node to the PriorityQueue with their path and their cost
			else if (neighbors.containsKey(currNode.name)) {
				for (Neighbor neighbor: neighbors.get(currNode.name)) {
					// Calculate cost with f(n) = g(n) + h(n) and add the node to the queue
					int pathCost = currNode.cost - straightLines.get(currNode.name) + neighbor.cost + straightLines.get(neighbor.name);
					nodeQueue.add(new Node(neighbor.name, currNode.path + ", " + neighbor.name, pathCost));
					
				} // End for (Neighbor neighbor: neighbors.get(currNode.name))
			} // End else if (neighbors.containsKey(currNode.name))
		} // End while (!nodeQueue.isEmpty())
		
		System.out.println("Path: " + bestPath + "\nCost: " + cost);
	}

	/**
	 * @param start
	 * @param goal
	 * @param neighbors
	 * @param straightLines
	 */
	public static void dStar(Node start, String goal, Map<String, List<Neighbor>> neighbors, Map<String, Integer> straightLines) {

	}
	
	/**
	 * @param fileName: The name of the file holding nodes, their neighbors, and the cost of traveling between them
	 * @return neighbors: A TreeMap that holds all the nodes, all their neighbors, and path cost of travel between them
	 * @throws FileNotFoundException
	 */
	public static Map<String, List<Neighbor>> getNeighbors(String fileName) throws FileNotFoundException {
		// Scanner to read cost of traveling between two directly connected nodes
		Scanner neighborsScan = new Scanner(new File(fileName));			
		
		// TreeMap to hold nodes and its neighbors
		Map<String, List<Neighbor>> neighbors = new TreeMap<String, List<Neighbor>>();	
		
		// Get every path between nodes from ActualDistance.txt
		while (neighborsScan.hasNextLine()) {	
			// Get node, neighbor, and path cost as  a string array
			String[] nodeAndNeighbor = neighborsScan.nextLine().split(" ");	
			
			// Make a neighbor object out of the neighbor
			Neighbor newNeighbor = new Neighbor(nodeAndNeighbor[1], Integer.parseInt(nodeAndNeighbor[2]));		
			
			// If that node name is not already in the neighbors map, add it
			if (!neighbors.containsKey(nodeAndNeighbor[0])) {		
				// Create a linked list to hold the nodes neighbors
				LinkedList<Neighbor> newNeighborList = new LinkedList<Neighbor>();	
				
				// Put the neighbor in the linked list
				newNeighborList.push(newNeighbor);	
				
				// Add the node with its single neighbor to the list
				neighbors.put(nodeAndNeighbor[0], newNeighborList);	
				
			} // End if (!neighbors.containsKey(nodeAndNeighbor[0]))
			
			// Else if that node is already in the list, add its new neighbor to its LinkedList
			else { neighbors.get(nodeAndNeighbor[0]).add(newNeighbor); }
			
		} // End while (neighborsScan.hasNextLine())

		neighborsScan.close();
		
		return neighbors;
	}
	
	/**
	 * @param fileName: The name of file holding the straight line distances from each city to the goal
	 * @return straightLines: A TreeMap holding the straight line distances from each city to the goal
	 * @throws FileNotFoundException
	 */
	public static TreeMap<String, Integer> getStraightLines(String fileName) throws FileNotFoundException {
		// Scanner to read distances from the file
		Scanner straightLineScan = new Scanner(new File("StraightLineDistances.txt"));	
		
		// TreeMap to hold those distances
		TreeMap<String, Integer> straightLines = new TreeMap<String, Integer>();
		
		// Parse the file
		while (straightLineScan.hasNextLine()) {
			// Make a string array of the node name and its distance from the goal
			String[] pointAndDistance = straightLineScan.nextLine().split(" ");	
			
			// Add node name and distance to the tree as a key value pair
			straightLines.put(pointAndDistance[0], Integer.parseInt(pointAndDistance[1]));	
			
		} // End while (straightLineScan.hasNextLine())
	
		straightLineScan.close();
		
		return straightLines;
	}

}
