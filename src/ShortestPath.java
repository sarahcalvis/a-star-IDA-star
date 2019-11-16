/*
 * COMP 422: Algorithms
 * Krause, Robosky, and Calvis
 * Using A* and D* to solve the shortest path problem with an admissible heuristic
 */

import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Scanner;
import java.util.TreeMap;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;

public class ShortestPath {
	static Node start;
	static String goal;
	static int numNodes;
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
		start = new Node("0", "0", 0);
		goal = "2";
		
		//////////////////////
		// Call and time A* //
		//////////////////////
		long startTime = System.nanoTime();
		aStar(start, goal, neighbors, straightLines);
		long endTime = System.nanoTime();
		System.out.println("A* finished executing on " + numNodes + " nodes in " + (endTime - startTime)/1000000000 + " seconds.");
		
		//////////////////////
		// Call and time D* //
		//////////////////////
		startTime = System.nanoTime();
		dStar(start, goal, neighbors, straightLines);
		endTime = System.nanoTime();
		System.out.println("D* finished executing on " + numNodes + " nodes in " + (endTime - startTime)/1000000009 + " seconds.");

	}

	/**
	 * @param start: The starting node
	 * @param goal: The goal node's name
	 * @param neighbors: A list that holds (node name, neighbor) pairs
	 * @param straightLines: A list of the straight line distances from all the nodes to the goal node
	 * Performs A*, a shortest path algorithm based on the Triangle Inequality
	 */
	public static void aStar(Node start, String goal, Map<String, List<Neighbor>> neighbors, Map<String, Integer> straightLines) {
		// Initialize to infinity because first solution found will be better
		int cost = -1;	
		String bestPath = "";			// Store the best path to goal

		NodeComparator compare = new NodeComparator();						// Use to compare nodes
		PriorityQueue<Node> nodeQueue = new PriorityQueue<Node>(compare);	// Holds nodes that must be traversed

		nodeQueue.add(start);			// Add the initial node

		// Iterate through the node queue
		while (!nodeQueue.isEmpty()) {
			Node currNode = nodeQueue.remove();	// Take the smallest node

			// If we found the goal
			if (currNode.name.equals(goal)) {	
				System.out.println("Found a path: " + currNode.path + " with cost " + currNode.cost);

				// If this path is better than the reigning path
				if (currNode.cost <= cost || cost == -1) {	
					cost = currNode.cost;		// Save path cost
					bestPath = currNode.path;	// Save path
					System.out.println("At cost " + currNode.cost + ", it's the best path so far.");
					// All the untraversed paths are worse than our solution therefore we found best path
					if (nodeQueue.peek().cost > cost) { 
						System.out.println("But "+nodeQueue.peek().path + " may be better, so we will keep traversing.");
						break;
					}
					else {
						System.out.println("Path: " + bestPath + "\nCost: " + cost);
						return;
					}
				} // End if (currNode.cost < cost)
			} // End if (currNode.name.equals(goal))

			// If the goal wasn't found, add the neighbors of the found node to the PriorityQueue with their path and their cost
			else if (neighbors.containsKey(currNode.name)) {
				for (Neighbor neighbor: neighbors.get(currNode.name)) {
					// Calculate cost with f(n) = g(n) + h(n) and add the node to the queue
					int pathCost = currNode.cost;
					pathCost -= straightLines.get(currNode.name);
					pathCost += neighbor.cost;
					pathCost += straightLines.get(neighbor.name);
					Node n = new Node(neighbor.name, currNode.path + ", " + neighbor.name, pathCost);
					//System.out.println(currNode.path + ", " + neighbor.name);
					nodeQueue.add(n);
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
		NodeComparator compare = new NodeComparator();						// Use to compare nodes
		PriorityQueue<Node> openList = new PriorityQueue<Node>(compare);					// Holds nodes that must be traversed

		openList.add(start);			// Add the initial node

		while(!neighbors.isEmpty()) {
			Node currNode = openList.remove();
			boolean isRaise = isRaise(currNode, neighbors);
			double cost;
			for (Neighbor neighbor: neighbors.get(currNode.name)) {
//				if(isRaise) {
//					if(neighbor.nextPoint.isSameNode(currNode)) {
//						neighbor.setNextPointAndUpdateCost(currNode);
//						int pathCost = currNode.cost - straightLines.get(currNode.name) + neighbor.cost + straightLines.get(neighbor.name);
//						openList.add(new Node(neighbor.name, currNode.path + ", " + neighbor.name, pathCost));
//					} 
//					else {
//						cost = neighbor.calculateCostVia(currNode);
//						if(cost < neighbor.cost) {
//							currNode.setMinimumCostToCurrentCost();
//							openList.add(currNode);
//						}
//					}
//				} else {
//					cost = neighbor.calculateCostVia(currNode);
//					if(cost < neighbor.getCost()) {
//						neighbor.setNextPointAndUpdateCost(currNode);
//						// TODO: probably have path cost wrong here
//						int pathCost = currNode.cost - straightLines.get(currNode.name) + neighbor.cost + straightLines.get(neighbor.name);
//						openList.add(new Node(neighbor.name, currNode.path + ", " + neighbor.name, pathCost));
//						openList.add(neighbor);
//					}
//				}
			}
		}
	}

	public static boolean isRaise(Node node,  Map<String, List<Neighbor>> neighbors) {
		int cost;
//		if(node.getCurrentCost() > node.getMinimumCost()) {
//			for (Neighbor neighbor: neighbors.get(node.name)) {
//				cost = node.calculateCostVia(neighbor);
//				if(cost < node.getCurrentCost()) {
//					node.setNextPointAndUpdateCost(neighbor);
//				}
//			}
//		}
//		return node.getCurrentCost() > node.getMinimumCost();
		return false;
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
				LinkedList<Neighbor> newNeighborList = new LinkedList<Neighbor>();	// Create a linked list to hold the nodes neighbors				
				newNeighborList.push(newNeighbor);									// Put the neighbor in the linked list
				neighbors.put(nodeAndNeighbor[0], newNeighborList);					// Add the node with its single neighbor to the list
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
		Scanner straightLineScan = new Scanner(new File(fileName));	

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
	
	/**
	 * Generate an unconnected graph
	 */
	public static void generateGraph() {
		Scanner s = new Scanner(System.in);
		numNodes = s.nextInt();
		GraphGenerator g = new GraphGenerator(numNodes);
		g.generateCities();
		g.generateConnections();
		g.writeToFile();
		System.out.println("Graph generated.");
	}
}
