import java.util.List;
import java.util.Map;
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
			Neighbor newNeighbor = new Neighbor(nodeAndNeighbor[1], Integer.parseInt(nodeAndNeighbor[2]), Integer.parseInt(nodeAndNeighbor[2]));
			
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
		
		//hard code in the initial node. This will always be the startest point when we calculate shortest path
		Node start = new Node("Ar", "Ar", 0);
		
		//call aStar
		aStar(start, neighbors, straightLines);
		
		//call dStar
		dStar(start, neighbors, straightLines);
	}
	public static void aStar(Node start, Map<String, List<Neighbor>> neighbors, TreeMap<String, Integer> straightLines) {
		
	}
	public static void dStar(Node start, Map<String, List<Neighbor>> neighbors, TreeMap<String, Integer> straightLines) {

	}
}
