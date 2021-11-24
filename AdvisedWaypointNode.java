import io.jbotsim.core.*;
import io.jbotsim.core.event.*;
import io.jbotsim.ui.JViewer;
import io.jbotsim.core.Point;

import java.util.LinkedList;
import java.util.Queue;
import java.util.Vector;

public class AdvisedWayPointNode extends Node{
	static public int nbTransmission = 0;
	static public int startingTime = 0;
	static public SinkNode Sink = new SinkNode ();
	
	double step = 1;
	
    public double distanceToSink = 0;
    public double traveledDistance = 0;
    public java.util.Vector<Point> destinations = new java.util.Vector<Point>();
    
    public int receivedData = 0;
    
    //destination
    private Point target;
    
    private Point origin = this.getLocation();
       
    private Point sink = new Point(300,300,0);
    
    public static Stat s;
    
    //general parameters for the equation of a line
    private double a,b,c;
    
	public void onStart(){
		this.traveledDistance = 0.0;
		this.distanceToSink = 0.0;
		
		this.setProperty("data", true);
		this.setProperty("distanceToSink", 0.0);
        this.setColor(Color.green);
		this.setCommunicationRange(30);
		
		this.setProperty("intersectsSink", false);
		
		this.destinations.clear();
		this.computeDestinations();
		
		s = new Stat(this.getTopology());
	}
	
	private void updateIntersectSink() {
		if( this.hasProperty("sink") && (boolean)this.getProperty("sink") ) return;
		
		//update a,b,c
		findGeneralEquationOfALine(origin,target);
		
		
		this.setProperty("intersectsSink", checkCollision(a,b,c));
		//System.out.println("Node "+this.getID()+" intersectsSink = "+ (boolean)this.getProperty("intersectsSink"));
	}
	
	//https://www.geeksforgeeks.org/check-line-touches-intersects-circle/
	private boolean checkCollision(double a, double b, double c)
	{
		double x = this.sink.getX();
		double y = this.sink.getY();
		double radius = this.getCommunicationRange();
		// Finding the distance of line from center.
				
		double dist = (Math.abs(a * x + b * y + c)) /
		     Math.sqrt(a * a + b * b);
		
		// Checking if the distance is less than,
		// greater than or equal to radius.
		if (radius >= dist ) return true;
		else return false;
		
	}
	
	//https://www.geeksforgeeks.org/program-find-line-passing-2-points/
	private void findGeneralEquationOfALine(Point P, Point Q) {
		a = Q.y - P.y;
        b = P.x - Q.x;
        c = a * (P.x) + b * (P.y);
	}
	
	private boolean doWeTransmitToNode(Node node)
	{
		//only send if and only if the node is going to the sink and the current node is not going to the sink.
		//obviously send if the node is the sink.
		return (boolean)node.hasProperty("intersectsSink") && (boolean)node.getProperty("intersectsSink") 
				&& !(boolean)this.getProperty("intersectsSink")
				|| node.hasProperty("sink") && (boolean)node.getProperty("sink");
	}

	public void onClock()
	{
		s.onClock();
		//If we don't have data, we cannot do anything, anyway we get data if the curr node has no data
		//but is going toward the sink.
		updateIntersectSink();
		if(!(boolean)this.getProperty("data") /*&& !(boolean)this.getProperty("intersectsSink")*/)
		{
			return;
		}

		java.util.List<Node> neigList = getNeighbors();
        for(Node node: neigList) {
            if((boolean)node.getProperty("data"))
            {
                if(doWeTransmitToNode(node))
                {
                	setProperty("data", false);
                	setColor(Color.red);
                	nbTransmission++;
                	break;
                }
            }
        }
        
        
        if(nbTransmission == this.getTopology().getNodes().size() - 1)
        {
        	System.out.println("Aggregation Done in "+(this.getTopology().getTime() - startingTime)+" time unit ");
        	nbTransmission = 0;
        	this.getTopology().restart();
        	startingTime = this.getTopology().getTime();
        }
        
        

	}

	public void onPreClock() {

        double remainingDistance = step; 
        
        while(true)
        {
        	if (destinations.size() == 0){
	            this.computeDestinations();
	        }
        	this.setDirection(destinations.get(0));
        	// If the next destination is close, we move to this destination and we continue to the next destination.
        	if(this.getLocation().distance(destinations.get(0)) < remainingDistance)
        	{
        		double advancedDistance = this.getLocation().distance(destinations.get(0));
        		remainingDistance -= advancedDistance;
        		this.setLocation(destinations.get(0));
        		destinations.remove(0);
        	}
        	else // Else, we move toward the next destination, by the remaining distance
        	{
        		this.move(remainingDistance);
        		break;
        	}
        }
        traveledDistance += step;
        if(this.traveledDistance + step >= this.distanceToSink)
        {
            this.computeDestinations();
            
        }
	}

    Point projection(Point v, Point w, Point p) {
        // Return minimum distance between line segment vw and point p
        double l2 = v.distance(w);
        if (l2 == 0.0) return v;   // v == w case
        // Consider the line extending the segment, parameterized as v + t (w - v).
        // We find projection of point p onto the line.
        // It falls where t = [(p-v) . (w-v)] / |w-v|^2
        double t = ((p.getX() - v.getX())*(w.getX() - v.getX()) +
                    (p.getY() - v.getY())*(w.getY() - v.getY())) / l2;
        if (t < 0.0) return v;       // Beyond the 'v' end of the segment
        if (t > 1.0) return w;  // Beyond the 'w' end of the segment

        Point projection = new Point(v.getX() + t * (w.getX() - v.getX()),
                                    v.getY() + t * (w.getY() - v.getY()));  // Projection falls on the segment
        return projection;
    }
    
    void computeDestinations()
    {
    	
    	
        origin = this.getLocation();
        boolean firstPass = true;
        
        if(destinations.size() > 0)
        {
            origin = destinations.lastElement();
            firstPass = false;
        }

        while(true)
        {
            target = new Point(Math.random()*600, Math.random()*600);
            destinations.add(target);
            
            Point proj = projection(origin, target, this.Sink.getLocation());

            if(proj.distance(this.Sink.getLocation()) < this.getCommunicationRange() - 1.0
            	&& (!firstPass ||
            		this.getLocation().distance(this.Sink.getLocation()) > this.getCommunicationRange()
            		)
            	)
            {
                this.distanceToSink += origin.distance(proj);
                this.setProperty("distanceToSink", this.distanceToSink);
                return;
            }
            firstPass = false;
            this.distanceToSink += origin.distance(target);
            origin = target;
            
        }
    }

}
