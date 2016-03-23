/**
 * @author UCSD MOOC development team and YOU
 * @author Vu Nguyen
 * Date: Mar 22, 2016
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.function.Consumer;

//import com.sun.javafx.geom.Edge;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 *         A class which represents a graph of geographic locations Nodes in the
 *         graph are intersections between
 *
 */
public class MapGraph {
	Map<GeographicPoint, Set<Edge>> vertices;
	int numEdges;
	// Set<Edge> edges;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		vertices = new HashMap<GeographicPoint, Set<Edge>>();
		numEdges = 0;
		// edges = new HashSet<Edge>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return vertices.size();
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		return vertices.keySet();
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		return numEdges;
	}

	public boolean contains(GeographicPoint p) {
		for (GeographicPoint v : vertices.keySet()) {
			if (v.equals(p)) {
				return true;
			}
		}

		return false;
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point If the
	 * location is already in the graph or null, this method does not change the
	 * graph.
	 * 
	 * @param location
	 *            The location of the intersection
	 * @return true if a node was added, false if it was not (the node was
	 *         already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		if (!this.contains(location)) {
			vertices.put(location, new HashSet<Edge>());
			return true;
		}

		return false;
	}

	public boolean containsEdge(GeographicPoint v, Edge e) {
		if (this.contains(v)) {
			for (Edge ed : vertices.get(v)) {
				if (ed.equals(e)) {
					return true;
				}
			}
		}
		return false;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2. Precondition: Both
	 * GeographicPoints have already been added to the graph
	 * 
	 * @param from
	 *            The starting point of the edge
	 * @param to
	 *            The ending point of the edge
	 * @param roadName
	 *            The name of the road
	 * @param roadType
	 *            The type of the road
	 * @param length
	 *            The length of the road, in km
	 * @throws IllegalArgumentException
	 *             If the points have not already been added as nodes to the
	 *             graph, if any of the arguments is null, or if the length is
	 *             less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
			throws IllegalArgumentException {
		this.addVertex(from);
		this.addVertex(to);

		if ((!this.contains(from) || !this.contains(to))
				|| (from == null || to == null || roadName == null || roadType == null) || length < 0) {
			throw new IllegalArgumentException();
		}

		Edge e = new Edge(from, to, roadName, roadType, length);
		if (!this.containsEdge(from, e)) {
			vertices.get(from).add(e);
		}
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// initialization
		if (this.contains(start) && this.contains(goal)) {
			Map<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();
			Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
			List<GeographicPoint> queue = new LinkedList<GeographicPoint>();

			queue.add(start);
			visited.add(start);
			parent.put(start, null);

			while (!queue.isEmpty()) {
				GeographicPoint curr = queue.remove(0);
				nodeSearched.accept(curr);

				if (curr.equals(goal)) {
					return getPath(parent, goal);
				}

				for (Edge e : vertices.get(curr)) {
					GeographicPoint neighbor = e.getTo();
					if (!visited.contains(neighbor)) {
						visited.add(neighbor);
						parent.put(neighbor, curr);
						queue.add(neighbor);
					}
				}
			}
		}

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());

		return null;
	}

	public Map<GeographicPoint, Double> initDistance() {
		Map<GeographicPoint, Double> dist = new HashMap<GeographicPoint, Double>();
		for (GeographicPoint p : vertices.keySet()) {
			dist.put(p, Double.MAX_VALUE);
		}
		return dist;
	}

	public List<GeographicPoint> getPath(Map<GeographicPoint, GeographicPoint> p, GeographicPoint g) {
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		path.add(g);
		GeographicPoint curr = p.get(g);

		while (curr != null) {
			path.add(curr);
			curr = p.get(curr);
		}
		// might not be necessary considering front-end is just
		// drawing the path (goal -> start ~ start -> goal)
		Collections.reverse(path);

		return path;
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// initialization
		if (this.contains(start) && this.contains(goal)) {

			PriorityQueue<Edge> pq = new PriorityQueue<Edge>();
			Map<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();
			Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
			Map<GeographicPoint, Double> dist = initDistance();

			dist.put(start, 0.);
			pq.add(new Edge(start, start, "", "", 0.));
			visited.add(start);
			parent.put(start, null);

			while (!pq.isEmpty()) {
				// remove curr from head of the priority queue
				GeographicPoint curr = pq.remove().getTo();

				// update Hook to Front-end
				nodeSearched.accept(curr);

				// build path if we found goal
				if (curr.equals(goal)) {
					return getPath(parent, goal);
				}

				// mark curr visited
				if (!visited.contains(curr)) {
					visited.add(curr);
				}

				for (Edge e : vertices.get(curr)) {
					GeographicPoint neighbor = e.getTo();
					// create a copy to put on pq because we dont want to alter
					// the edges
					Edge copy = e.createCopy();

					// if we haven't visited n and
					// if the path to n (thru curr) is shorter
					if (!visited.contains(neighbor) && (dist.get(curr) + copy.length() < dist.get(neighbor))) {
						parent.put(neighbor, curr);
						dist.put(neighbor, dist.get(curr) + e.length());
						copy.setLength(dist.get(neighbor));
						pq.add(copy);
					}
				}
			}
		}
		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());

		return null;
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {

		// initialization
		if (this.contains(start) && this.contains(goal)) {

			PriorityQueue<Edge> pq = new PriorityQueue<Edge>();
			Map<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();
			Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
			Map<GeographicPoint, Double> dist = initDistance();
			Map<GeographicPoint, Double> est = initDistance();

			dist.put(start, 0.);
			est.put(start, start.distance(goal));
			pq.add(new Edge(start, start, "", "", est.get(start)));
			visited.add(start);
			parent.put(start, null);

			while (!pq.isEmpty()) {
				// remove curr from head of the priority queue
				GeographicPoint curr = pq.remove().getTo();

				// update Hook to Front-end
				nodeSearched.accept(curr);

				// build path if we found goal
				if (curr.equals(goal)) {
					return getPath(parent, goal);
				}

				// mark curr visited
				if (!visited.contains(curr)) {
					visited.add(curr);
				}

				for (Edge e : vertices.get(curr)) {
					GeographicPoint neighbor = e.getTo();
					// create a copy to put on pq because we dont want to alter
					// the edges
					Edge copy = e.createCopy();

					// if we haven't visited n and
					// if the path to n (thru curr) is shorter
					if (!visited.contains(neighbor) && (dist.get(curr) + copy.length() < dist.get(neighbor))) {
						parent.put(neighbor, curr);
						dist.put(neighbor, dist.get(curr) + e.length());
						est.put(neighbor, dist.get(neighbor) + neighbor.distance(goal));
						copy.setLength(est.get(neighbor));
						pq.add(copy);
					}
				}
			}
		}

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());

		return null;
	}

	public static void main(String[] args) {
		/*System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");*/

		// You can use this method for testing.

		// Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start, end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start, end);
	}

}
