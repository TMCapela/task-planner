package meta_csp_planner;

import java.util.logging.Level;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Iterator;
import java.io.*;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.meta.simplePlanner.SimpleDomain;
import org.metacsp.meta.simplePlanner.SimpleDomain.markings;
import org.metacsp.meta.simplePlanner.SimpleOperator;
import org.metacsp.meta.simplePlanner.SimplePlanner;
import org.metacsp.meta.symbolsAndTime.Schedulable;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.spatial.utility.SpatialRule;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.logging.MetaCSPLogging;
import org.metacsp.utility.timelinePlotting.TimelinePublisher;
import org.metacsp.utility.timelinePlotting.TimelineVisualizer;

public class MySimpleTest {
	public static void main(String[] args) throws IOException {
		
		SimplePlanner planner = new SimplePlanner(0, 1000, 0);
		
		SimpleDomain.parseDomain(planner, "./domains/MyTestDomain.dll", SimpleDomain.class);
		
		// This is a pointer toward the ground constraint network of the planner
		ActivityNetworkSolver groundSolver = (ActivityNetworkSolver)planner.getConstraintSolvers()[0];

		// INITIAL AND GOAL STATE DEFS
		SymbolicVariableActivity one = (SymbolicVariableActivity)groundSolver.createVariable("RobotAction");
		one.setSymbolicDomain("TakePictureFoV4()");
		// ... this is a goal (i.e., an activity to justify through the meta-constraint)
		one.setMarking(markings.UNJUSTIFIED);
		//.. let's also give it a minimum duration
		AllenIntervalConstraint durationOne = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(7,APSPSolver.INF));
		durationOne.setFrom(one);
		durationOne.setTo(one);

		SymbolicVariableActivity two = (SymbolicVariableActivity)groundSolver.createVariable("RobotAction");
		two.setSymbolicDomain("TakePictureFoV3()");
		two.setMarking(markings.UNJUSTIFIED);
		AllenIntervalConstraint durationTwo = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(7,APSPSolver.INF));
		durationTwo.setFrom(two);
		durationTwo.setTo(two);
		
		SymbolicVariableActivity three = (SymbolicVariableActivity)groundSolver.createVariable("RobotAction");
		three.setSymbolicDomain("TakePictureFoV2()");
		// ... this is a goal (i.e., an activity to justify through the meta-constraint)
		three.setMarking(markings.UNJUSTIFIED);
		//.. let's also give it a minimum duration
		AllenIntervalConstraint durationThree = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(7,APSPSolver.INF));
		durationThree.setFrom(three);
		durationThree.setTo(three);

		SymbolicVariableActivity four = (SymbolicVariableActivity)groundSolver.createVariable("RobotAction");
		four.setSymbolicDomain("TakePictureFoV1()");
		four.setMarking(markings.UNJUSTIFIED);
		AllenIntervalConstraint durationFour = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(7,APSPSolver.INF));
		durationFour.setFrom(four);
		durationFour.setTo(four);

		//groundSolver.addConstraints(new Constraint[] {durationOne, durationTwo, durationThree, durationFour});
		
		// We can also specify that goals should be related in time somehow...		
		AllenIntervalConstraint after = new AllenIntervalConstraint(AllenIntervalConstraint.Type.After, AllenIntervalConstraint.Type.After.getDefaultBounds());
		after.setFrom(one);
		after.setTo(two);
		groundSolver.addConstraint(after);

		//TimelinePublisher tp = new TimelinePublisher(groundSolver, "Robot1", "Robot2", "LocalizationService", "RFIDReader1", "LaserScanner1");
		TimelinePublisher tp = new TimelinePublisher(groundSolver.getConstraintNetwork(), "atLocation", "RobotAction", "RobotProprioception");
		TimelineVisualizer viz = new TimelineVisualizer(tp);
		tp.publish(false, false);
		
		planner.backtrack();
		
		ConstraintNetwork.draw(groundSolver.getConstraintNetwork(), "Constraint Network");
		
		planner.draw();
		tp.publish(true, false);
		
		//#####################################################################################################################
		//sort Activity based on the start time for debugging purpose
		HashMap<SymbolicVariableActivity, Long> starttimes = new HashMap<SymbolicVariableActivity, Long>();
		for (int i = 0; i < groundSolver.getVariables().length; i++) {
			starttimes.put((SymbolicVariableActivity) groundSolver.getVariables()[i], ((SymbolicVariableActivity)groundSolver.getVariables()[i]).getTemporalVariable().getStart().getLowerBound());                       
		}
		
		BufferedWriter writer = new BufferedWriter(new FileWriter("./output/SimpleTestOut.txt"));
		
		//          Collections.sort(starttimes.values());
		starttimes =  sortHashMapByValuesD(starttimes);
		for (SymbolicVariableActivity act : starttimes.keySet()) {
			System.out.println(act + " --> " + starttimes.get(act));
			writer.write(act + " --> " + starttimes.get(act) + "\n");
		}
		writer.close();	    
	    
		//#####################################################################################################################

	}
	
	private static LinkedHashMap sortHashMapByValuesD(HashMap passedMap) {
		ArrayList mapKeys = new ArrayList(passedMap.keySet());
		ArrayList mapValues = new ArrayList(passedMap.values());
		Collections.sort(mapValues);
		Collections.sort(mapKeys);

		LinkedHashMap sortedMap =  new LinkedHashMap();

		Iterator valueIt = ((java.util.List<SpatialRule>) mapValues).iterator();
		while (valueIt.hasNext()) {
			long val = (Long) valueIt.next();
			Iterator keyIt = ((java.util.List<SpatialRule>) mapKeys).iterator();

			while (keyIt.hasNext()) {
				SymbolicVariableActivity key = (SymbolicVariableActivity) keyIt.next();
				long comp1 = (Long) passedMap.get(key);
				long comp2 = val;

				if (comp1 == comp2){
					passedMap.remove(key);
					mapKeys.remove(key);
					sortedMap.put(key, val);
					break;
				}
			}
		}
		return sortedMap;
	}
}
