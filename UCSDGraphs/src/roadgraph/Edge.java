/**
 * @author Vu Nguyen
 * Date: Mar 22, 2016
 * A class which represents a edge on the graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */

package roadgraph;

import geography.GeographicPoint;

public class Edge {
	GeographicPoint from; 
	GeographicPoint to; 
	String roadName;
	String roadType; 
	double length;
	
	public Edge() {
	}
		
	public Edge(GeographicPoint from, GeographicPoint to, String rn, String rt, double length) {
		this.from = from;
		this.to = to;
		this.roadName = rn;
		this.roadType = rt;
		this.length = length;
	}
	
	public void setRoadName(String n) {
		roadName = n;
	}
	
	public void setRoadType(String t) {
		roadType = t;
	}
	
	public GeographicPoint getFrom() {
		return from;
	}
	
	public GeographicPoint getTo() {
		return to;
	}
	
	public boolean equals(Edge e) {
		return (this.from == e.getFrom() && this.to== e.getTo());
	}
}
