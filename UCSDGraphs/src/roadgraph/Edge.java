/**
 * @author Vu Nguyen
 * Date: Mar 22, 2016
 * A class which represents a edge on the graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */

package roadgraph;

import geography.GeographicPoint;

public class Edge implements Comparable<Edge> {
	GeographicPoint from; 
	GeographicPoint to; 
	String roadName;
	String roadType; 
	double length;
	boolean multiple;
	
	public Edge() {
	}
		
	public Edge(GeographicPoint from, GeographicPoint to, String rn, String rt, double length) {
		this.from = from;
		this.to = to;
		this.roadName = rn;
		this.roadType = rt;
		this.length = length;
		this.multiple = false;
	}
	
	public Edge(GeographicPoint from, GeographicPoint to, String rn, String rt, double length, boolean multiple) {
		this.from = from;
		this.to = to;
		this.roadName = rn;
		this.roadType = rt;
		this.length = length;
		this.multiple = multiple;
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
	
	public double length() {
		return length;
	}
	
	public boolean getMult() {
		return multiple;
	}
	
	public void setMult(boolean val) {
		this.multiple = val;
	}
	
	public void setLength(double l) {
		length += l;
	}
	
	public boolean equals(Edge e) {
		return (this.from == e.getFrom() && this.to == e.getTo()) && (this.multiple == e.getMult());
	}
	
	public int compareTo(Edge other) {
		if (this.length() < other.length()) {
			return -1;
		} else if (this.length() > other.length()) {
			return 1;
		} else {
			return 0;
		}
	}
	
	public Edge createCopy() {
		return new Edge(this.getFrom(), this.getTo(), this.roadName, this.roadType, this.length(), this.getMult());
	}
}
