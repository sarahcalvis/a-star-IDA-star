import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Scanner;
import java.util.TreeMap;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.Arrays;
import java.util.LinkedList;
public class Main {
	public static void main(String[] args) throws FileNotFoundException {

		//ActualDistances.txt holds the cost of traveling between two directly connected nodes
		Scanner neighborsScan = new Scanner(new File("ActualDistances.txt"));

		//this map's key is the node name and its value is the neighbors of said node
		Map<String, List<Neighbor>> neighbors = new TreeMap<String, List<Neighbor>>();

		//get every path between nodes from ActualDistance.txt
		while (neighborsScan.hasNextLine()) {
			//get one node name, neighbor name, path cost combination and split it into a string array
			String[] nodeAndNeighbor = neighborsScan.nextLine().split(" ");

			//make a neighbor object out of the neighbor
			Neighbor newNeighbor = new Neighbor(nodeAndNeighbor[1], Integer.parseInt(nodeAndNeighbor[2]));

			//if that node name is not already in the neighbors map, add it
			if (!neighbors.containsKey(nodeAndNeighbor[0])) {

				//create a linked list to hold the nodes neighbors
				LinkedList<Neighbor> newNeighborList = new LinkedList<Neighbor>();

				//put the neighbor in the linked list
				newNeighborList.push(newNeighbor);

				//add the node with its single neighbor to the list
				neighbors.put(nodeAndNeighbor[0], newNeighborList);
			}

			//else if that node is already in the list, add its new neighbor
			else {

				//add the neighbor to the node's linked list
				neighbors.get(nodeAndNeighbor[0]).add(newNeighbor);
			}

		}

		neighborsScan.close();

		//read the straight line distance from each node to the goal
		Scanner straightLineScan = new Scanner(new File("StraightLineDistances.txt"));

		//TreeMap to hold node names and their straight line distance to the goal
		TreeMap<String, Integer> straightLines = new TreeMap<String, Integer>();

		//fill the TreeMap
		while (straightLineScan.hasNextLine()) {

			//make a string array of the node name and its distance from the goal
			String[] pointAndDistance = straightLineScan.nextLine().split(" ");

			//add them to the tree as a key value pair
			straightLines.put(pointAndDistance[0], Integer.parseInt(pointAndDistance[1]));
		}

		straightLineScan.close();

		//hard code the initial node. This will always be the startest point when we calculate shortest path
		Node start = new Node("Ar", "Ar", 0);

		//hard code the goal node name
		String goal = "Bu";

		//call aStar
		aStar(start, goal, neighbors, straightLines);

		//call dStar
		dStar(start, goal, neighbors, straightLines);
	}

	public static void aStar(Node start, String goal, Map<String, List<Neighbor>> neighbors, TreeMap<String, Integer> straightLines) {
		//initialize the path cost to the biggest cost possible because we know the first solution we find will be better than this
		int cost = Integer.MAX_VALUE;

		//stores the best possible path to goal
		String bestPath = "";

		//make a Comparator for the PriorityQueue
		NodeComparator nodeComparator = new NodeComparator();

		//make a PriorityQueue to hold nodes that must be traversed
		PriorityQueue<Node> nodeQueue = new PriorityQueue<Node>(nodeComparator);

		//add the initial node
		nodeQueue.add(start);

		//until the PriorityQueue is empty
		while (!nodeQueue.isEmpty()) {

			//take the first node from nodeOptions
			Node currNode = nodeQueue.remove();

			//if we found the goal
			if (currNode.name.equals(goal)) {

				/* if this path is better than the reigning path 
				 * (we initialized cost to MAX_VALUE so we know the first path will be better than the initial cost) 
				 */
				if (currNode.cost < cost) {

					//save the best cost and path
					cost = currNode.cost;
					bestPath = currNode.path;

					//if all the untraversed paths are already worse than our solution, we are done and break out of the loop
					if (nodeQueue.peek().cost > cost) { break; }
				}
			}
			//if the goal wasn't found, add the neighbors of the found node to the PriorityQueue with their path and their cost
			else if (neighbors.containsKey(currNode.name)) {
				for (Neighbor neighbor: neighbors.get(currNode.name)) {

					/* we calculate cost with f(n) = g(n) + h(n)
					 * g(n) = cost to get to this node
					 * h(n) = estimated cost from the current node to the goal
					 * the cost of currNode is f(currNode) = g(currNode) + h(currNode)
					 * we want to calculate f(newNode) = g(newNode) + h(newNode)
					 * h(newNode) is stored in straightLines
					 * g(newNode) = g(currNode) + neighbor.cost
					 * now, we aren't storing g(currNode)
					 * we find it by doing this calculation: g(currNode) = f(currNode) - h(currNode)
					 * f(currNode) is just currNode.cost, and h(currNode) is stored in straightLines
					 * so f(newNode) = f(currNode) - h(currNode) + neighbor.cost + h(newNode)
					 */
					int pathCost = currNode.cost - straightLines.get(currNode.name) + neighbor.cost + straightLines.get(neighbor.name);

					//add the node to the queue
					nodeQueue.add(new Node(neighbor.name, currNode.path + ", " + neighbor.name, pathCost));
				}
			}
		}
		System.out.println("Path: " + bestPath + "\nCost: " + cost);
	}

	public static void dStar(Node start, String goal, Map<String, List<Neighbor>> neighbors, TreeMap<String, Integer> straightLines) {

	}
}
