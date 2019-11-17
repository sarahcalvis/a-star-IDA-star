///////////////////////////////////////////////////////////////////////////////////////
// COMP 422: Algorithms ///////////////////////////////////////////////////////////////
// Krause, Robosky, and Calvis ////////////////////////////////////////////////////////
// Using A* and IDA* to solve the shortest path problem with an admissible heuristic //
///////////////////////////////////////////////////////////////////////////////////////

import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Scanner;
import java.util.TreeMap;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.LinkedList;

public class ShortestPath {
	static int numNodes;
	static String soln;
	/**
	 * @param args
	 * @throws FileNotFoundException
	 * Sets up for A* and D* and then performs these searches
	 */
	public static void main (String[] args) throws FileNotFoundException {
		/////////////////////////////////////////////////////
		// Generate connected graph of user-specified size //
		/////////////////////////////////////////////////////
		generateGraph();

		///////////////////////////////////////////////////////
		// Read nodes, heuristic costs, neighbors from files //
		///////////////////////////////////////////////////////
		Map<String, Integer> straightLines = getStraightLines("points.txt");
		Map<String, List<Neighbor>> neighbors = getNeighbors("connections.txt");

		////////////////////////////////////
		// Hard code start and goal nodes //
		////////////////////////////////////
		Node start = new Node("0", "0", 0);
		String goal = "1";

		//////////////////////
		// Call and time A* //
		//////////////////////
		long startTime = System.nanoTime();
		String aStarSolution = aStar(start, goal, neighbors, straightLines);
		long endTime = System.nanoTime();
		System.out.println("A* found solution:" + aStarSolution + ".");
		System.out.println("A* found the shortest path in a graph of size " + numNodes + " nodes in " + (endTime - startTime)/1000000000 + " seconds.\n");

		//////////////////////
		// Call and time D* //
		//////////////////////
		startTime = System.nanoTime();
		String IDAStarSolution = IDA(start, goal, neighbors, straightLines);
		endTime = System.nanoTime();
		System.out.println("IDA* found solution:" + IDAStarSolution + ".");
		System.out.println("IDA* found the shortest path in a graph of size " + numNodes + " nodes in " + (endTime - startTime)/1000000009 + " seconds.\n");

	}

	/**
	 * @param start: The starting node
	 * @param goal: The goal node's name
	 * @param neighbors: A list that holds (node name, neighbor) pairs
	 * @param straightLines: A list of the straight line distances from all the nodes to the goal node
	 * Performs A*, a shortest path algorithm based on the Triangle Inequality
	 */
	public static String aStar(Node start, String goal, Map<String, List<Neighbor>> neighbors, Map<String, Integer> straightLines) {
		////////////////
		// Initialize //
		////////////////
		int cost = -1;														// Initialize to infinity because first solution found will be better
		String bestPath = "";												// Store the best path to goal
		NodeComparator compare = new NodeComparator();						// Use to compare nodes
		PriorityQueue<Node> nodeQueue = new PriorityQueue<Node>(compare);	// Holds nodes that must be traversed
		start.cost = straightLines.get(start.name);							// This implementation assumes nodes come with their heuristic cost factored in
		nodeQueue.add(start);												// Add the initial node
		
		///////////////////////////////////
		// Iterate through the node queue//
		///////////////////////////////////
		while (!nodeQueue.isEmpty()) {
			Node currNode = nodeQueue.remove();	// Take the smallest node off of the queue
			
			////////////////////////////////////////////////////////////
			// If we found a path to the goal, compare to prior paths //
			////////////////////////////////////////////////////////////
			if (currNode.name.equals(goal)) {	
				if (currNode.cost <= cost || cost == -1) {	// If this path is better than the reigning path
					cost = currNode.cost;					// Save path cost
					bestPath = currNode.path;				// Save path
					
					//////////////////////////////////////////////////////////////////////////////////
					// If all the untraversed paths are worse than our solution, we found best path //
					//////////////////////////////////////////////////////////////////////////////////
					if (nodeQueue.peek().cost > cost) return "\n\tPath: " + bestPath + "\n\tCost: " + cost;
					
				} // End if (currNode.cost < cost)
			} // End if (currNode.name.equals(goal))

			//////////////////////////////////////////////////////
			// If the goal wasn't found, add neighbors to queue //
			//////////////////////////////////////////////////////
			else if (neighbors.containsKey(currNode.name)) {
				for (Neighbor neighbor: neighbors.get(currNode.name)) {
					
					//////////////////////////////////////////////////////////////////////////
					// Calculate cost with f(n) = g(n) + h(n) and add the node to the queue //
					//////////////////////////////////////////////////////////////////////////
					int pathCost = currNode.cost - straightLines.get(currNode.name) + neighbor.cost + straightLines.get(neighbor.name);
					nodeQueue.add(new Node(neighbor.name, currNode.path + ", " + neighbor.name, pathCost));
					
				} // End for (Neighbor neighbor: neighbors.get(currNode.name))
			} // End else if (neighbors.containsKey(currNode.name))
		} // End while (!nodeQueue.isEmpty())
		
		///////////////////////////////////////////////////////////////////
		// If we iterated through all the nodes return the best solution //
		///////////////////////////////////////////////////////////////////
		return "\n\tPath: " + bestPath + "\n\tCost: " + cost;
	}

	

	/**
	 * Performs Iterative Deepening A* on a connected graph.
	 * @param start: The starting node.
	 * @param goal: The goal node.
	 * @param neighbors: A list of the neighbors of each node.
	 * @param straightLines: The heuristic cost from each node to the goal.
	 * @return: A string holding the best path to the goal and the cost to the goal.
	 */
	static String IDA(Node start, String goal, Map<String, List<Neighbor>> neighbors, Map<String, Integer> straightLines) {
		////////////////
		// Initialize //
		////////////////
		ArrayList<Node> path = new ArrayList<Node>();	// Holds nodes along the path to the goal
		int bound = straightLines.get(start.name);		// Stop searching if we reach this bounds
		path.add(start);								// Add the starting node to the path
		int state = 0;									// Holds the state of our search
		
		////////////////////////////////////////////////////////
		// Keep performing searches until a solution is found //
		////////////////////////////////////////////////////////
		while(soln !="" && state != -1 && state != Integer.MAX_VALUE) {
			state = search(path, goal, 0, bound, neighbors, straightLines);	// Perform a search
			if (state == -1) break;											// Solution found
			if (state == Integer.MAX_VALUE) break;							// No solution
			bound = state;													// Search exceeded bounds; extend bounds
		}
		
		///////////////////////////////////////////////////////////////////////////
		// If we left the above loop, either there is no solution or we found it //
		///////////////////////////////////////////////////////////////////////////
		return soln;
	}

	/**
	 * Performs a search for Iterative Deepening A*
	 * @param path: Path to current node
	 * @param goal: Name of goal node
	 * @param costSoFar: Path cost to current node
	 * @param bound: Bounds of search
	 * @param neighbors: List of edges between nodes
	 * @param straightLines: Heuristic cost of each node
	 * @return: The state of the search
	 */
	static int search(ArrayList<Node> path, String goal, int costSoFar, int bound, Map<String, List<Neighbor>> neighbors, Map<String, Integer> straightLines) {
		////////////////
		// Initialize //
		////////////////
		Node currNode = path.get(path.size() - 1);				// Select the top node to traverse
		int f = costSoFar + straightLines.get(currNode.name);	// Calculate heuristic cost of that node
		
		//////////////////////////////////////////////////////////
		// If path is too expensive, return and increase bounds //
		//////////////////////////////////////////////////////////
		if (f > bound) return f;
		
		////////////////////////////////////////
		// If path found, return with triumph //
		////////////////////////////////////////
		if (currNode.name.equals(goal)) {
			soln = "\n\tPath: " + currNode.path + "\n\tCost: " + currNode.cost;
			return -1;
		}
		
		//////////////////////////////////////////////
		// Otherwise, traverse nodes along the path //
		//////////////////////////////////////////////
		int min = Integer.MAX_VALUE;
		for (Neighbor neighbor: neighbors.get(currNode.name)) {
			
			////////////////////////////////
			// Copy path to a new list, p //
			////////////////////////////////
			ArrayList<Node> p = new ArrayList<Node>(path);
			
			///////////////////////////////////////////////////////
			// Find out whether this node is already in the path //
			///////////////////////////////////////////////////////
			boolean inPath = false;
			for (Node n: path) if (n.name.equals(neighbor.name)) inPath = true;
			
			///////////////////////////////////////////////////////////////////////////////
			// If the node is not in the path, search it and return the new search state //
			///////////////////////////////////////////////////////////////////////////////
			if (!inPath) {
				p.add(new Node(neighbor.name, currNode.path + ", " + neighbor.name, currNode.cost + neighbor.cost - straightLines.get(currNode.name)+straightLines.get(neighbor.name)));
				int state = search(p, goal, costSoFar + neighbor.cost, bound, neighbors, straightLines);
				if (state == -1) return -1;
				if (state < min) min = state;
				p.remove(p.size() - 1);
			}
		}
		return min;
	}

	/**
	 * @param fileName: The name of the file holding nodes, their neighbors, and the cost of traveling between them
	 * @return neighbors: A TreeMap that holds all the nodes, all their neighbors, and path cost of travel between them
	 * @throws FileNotFoundException
	 */
	public static Map<String, List<Neighbor>> getNeighbors(String fileName) throws FileNotFoundException {
		////////////////
		// Initialize //
		////////////////
		Scanner neighborsScan = new Scanner(new File(fileName));						// Scanner to read cost of traveling between two directly connected nodes
		Map<String, List<Neighbor>> neighbors = new TreeMap<String, List<Neighbor>>();	// TreeMap to hold nodes and its neighbors

		//////////////////////////////////////////////////////////
		// Get every path between nodes from ActualDistance.txt //
		//////////////////////////////////////////////////////////
		while (neighborsScan.hasNextLine()) {	
			String[] nodeAndNeighbor = neighborsScan.nextLine().split(" ");									// Get node, neighbor, and path cost as  a string array
			Neighbor newNeighbor = new Neighbor(nodeAndNeighbor[1], Integer.parseInt(nodeAndNeighbor[2]));	// Make a neighbor object out of the neighbor

			///////////////////////////////////////////////////
			// If that node is not in neighbors list, add it //
			///////////////////////////////////////////////////
			if (!neighbors.containsKey(nodeAndNeighbor[0])) {		
				LinkedList<Neighbor> newNeighborList = new LinkedList<Neighbor>();	// Create a linked list to hold the nodes neighbors				
				newNeighborList.push(newNeighbor);									// Put the neighbor in the linked list
				neighbors.put(nodeAndNeighbor[0], newNeighborList);					// Add the node with its single neighbor to the list
			} // End if (!neighbors.containsKey(nodeAndNeighbor[0]))
			
			///////////////////////////////////////////////////////////////////////
			// Else if that node is in  list, add new neighbor to its LinkedList //
			///////////////////////////////////////////////////////////////////////
			else neighbors.get(nodeAndNeighbor[0]).add(newNeighbor); 

		} // End while (neighborsScan.hasNextLine())
		neighborsScan.close();
		
		//////////////////////////
		// Return the neighbors //
		//////////////////////////
		return neighbors;
	}

	/**
	 * @param fileName: The name of file holding the straight line distances from each city to the goal
	 * @return straightLines: A TreeMap holding the straight line distances from each city to the goal
	 * @throws FileNotFoundException
	 */
	public static TreeMap<String, Integer> getStraightLines(String fileName) throws FileNotFoundException {
		////////////////
		// Initialize //
		////////////////
		Scanner straightLineScan = new Scanner(new File(fileName));					// Scanner to read distances from the file
		TreeMap<String, Integer> straightLines = new TreeMap<String, Integer>();	// TreeMap to hold those distances

		//////////////////////////////
		// Parse file and fill tree //
		//////////////////////////////
		while (straightLineScan.hasNextLine()) {
			String[] pointAndDistance = straightLineScan.nextLine().split(" ");				// Make a string array of the node name and its distance from the goal
			straightLines.put(pointAndDistance[0], Integer.parseInt(pointAndDistance[1]));	// Add node name and distance to the tree as a key value pair
		} // End while (straightLineScan.hasNextLine())
		straightLineScan.close();
		
		////////////////////////////////////
		// Return straight line distances //
		////////////////////////////////////
		return straightLines;
	}

	/**
	 * Generate an unconnected graph
	 */
	public static void generateGraph() {
		///////////////////////////////////////
		// Take user input to get graph size //
		///////////////////////////////////////
		System.out.println("Input number of nodes in graph: ");
		Scanner s = new Scanner(System.in);
		numNodes = s.nextInt();
		s.close();
		
		//////////////////////////////
		// Call the graph generator //
		//////////////////////////////
		GraphGenerator g = new GraphGenerator(numNodes);
		g.generateCities();
		g.generateConnections();
		g.writeToFile();
		System.out.println("Graph generated.\n");
	}
}
