package meta_csp_planner;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.logging.Level;

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

public class PlannerTurtleWithoutDomain {
	
	public static void main(String[] args) throws IOException {
		MetaCSPLogging.setLevel(TimelinePublisher.class, Level.FINEST);

		SimplePlanner planner = new SimplePlanner(0,600,0);		
		// This is a pointer toward the ActivityNetwork solver of the Scheduler
		ActivityNetworkSolver groundSolver = (ActivityNetworkSolver)planner.getConstraintSolvers()[0];

		MetaCSPLogging.setLevel(planner.getClass(), Level.FINEST);
		MetaCSPLogging.setLevel(SimpleDomain.class, Level.FINEST);
				
		SimpleDomain rd = new SimpleDomain(new int[] {1}, new String[] {"robot"}, "TestDomain");

		//State which state variables are actions
		rd.addActuator("Robot1");
		rd.addActuator("Battery");
		rd.addActuator("Location");
		
		// Here I create two AllenIntervalConstraint for use in the operator I will define
		AllenIntervalConstraint durationMoveTo = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(5,APSPSolver.INF));		
		AllenIntervalConstraint durationSpin = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3,APSPSolver.INF));		
		AllenIntervalConstraint durationCharged = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(5,20));
		AllenIntervalConstraint moveToDuringCharged = new AllenIntervalConstraint(AllenIntervalConstraint.Type.During, AllenIntervalConstraint.Type.During.getDefaultBounds());
		AllenIntervalConstraint SpinAfterMoveToWP = new AllenIntervalConstraint(AllenIntervalConstraint.Type.MetBy, AllenIntervalConstraint.Type.MetBy.getDefaultBounds());
		AllenIntervalConstraint ChargedAfterCharging = new AllenIntervalConstraint(AllenIntervalConstraint.Type.MetBy, AllenIntervalConstraint.Type.MetBy.getDefaultBounds());
		AllenIntervalConstraint ChargingAfterMoveToC = new AllenIntervalConstraint(AllenIntervalConstraint.Type.MetBy, AllenIntervalConstraint.Type.MetBy.getDefaultBounds());

		
		// New operator: the first parameter is the name, the second are the constraints,
		// the third are requirement activities, fourth means
		// no usage of resources
		SimpleOperator operatorSpin1 = new SimpleOperator("Robot1::Spin1()",
				new AllenIntervalConstraint[] {SpinAfterMoveToWP},
				new String[] {"Robot1::MoveTo1()"},
				new int[] {1});
		// We can add constraints to the operator even after it has been created
		// this is useful for adding unary constraints on the head (which has index 0)
		operatorSpin1.addConstraint(durationSpin, 0, 0);
		rd.addOperator(operatorSpin1);
		
		SimpleOperator operatorSpin2 = new SimpleOperator("Robot1::Spin2()",
				new AllenIntervalConstraint[] {SpinAfterMoveToWP},
				new String[] {"Robot1::MoveTo2()"},
				new int[] {1});
		operatorSpin2.addConstraint(durationSpin, 0, 0);
		rd.addOperator(operatorSpin2);
		
		SimpleOperator operatorSpin3 = new SimpleOperator("Robot1::Spin3()",
				new AllenIntervalConstraint[] {SpinAfterMoveToWP},
				new String[] {"Robot1::MoveTo3()"},
				new int[] {1});
		operatorSpin3.addConstraint(durationSpin, 0, 0);
		rd.addOperator(operatorSpin3);
		
		SimpleOperator operatorSpin4 = new SimpleOperator("Robot1::Spin4()",
				new AllenIntervalConstraint[] {SpinAfterMoveToWP},
				new String[] {"Robot1::MoveTo4()"},
				new int[] {1});
		operatorSpin4.addConstraint(durationSpin, 0, 0);
		rd.addOperator(operatorSpin4);
		
		SimpleOperator operatorSpin5 = new SimpleOperator("Robot1::Spin5()",
				new AllenIntervalConstraint[] {SpinAfterMoveToWP},
				new String[] {"Robot1::MoveTo5()"},
				new int[] {1});
		operatorSpin5.addConstraint(durationSpin, 0, 0);
		rd.addOperator(operatorSpin5);
		
		SimpleOperator operatorSpin6 = new SimpleOperator("Robot1::Spin6()",
				new AllenIntervalConstraint[] {SpinAfterMoveToWP},
				new String[] {"Robot1::MoveTo6()"},
				new int[] {1});
		operatorSpin6.addConstraint(durationSpin, 0, 0);
		rd.addOperator(operatorSpin6);
		
		SimpleOperator operatorSpin7 = new SimpleOperator("Robot1::Spin7()",
				new AllenIntervalConstraint[] {SpinAfterMoveToWP},
				new String[] {"Robot1::MoveTo7()"},
				new int[] {1});
		operatorSpin7.addConstraint(durationSpin, 0, 0);
		rd.addOperator(operatorSpin7);
		
		SimpleOperator operatorSpin8 = new SimpleOperator("Robot1::Spin8()",
				new AllenIntervalConstraint[] {SpinAfterMoveToWP},
				new String[] {"Robot1::MoveTo8()"},
				new int[] {1});
		operatorSpin8.addConstraint(durationSpin, 0, 0);
		rd.addOperator(operatorSpin8);
		
		SimpleOperator operatorSpin9 = new SimpleOperator("Robot1::Spin9()",
				new AllenIntervalConstraint[] {SpinAfterMoveToWP},
				new String[] {"Robot1::MoveTo9()"},
				new int[] {1});
		operatorSpin9.addConstraint(durationSpin, 0, 0);
		rd.addOperator(operatorSpin9);
		
		SimpleOperator operatorSpin10 = new SimpleOperator("Robot1::Spin10()",
				new AllenIntervalConstraint[] {SpinAfterMoveToWP},
				new String[] {"Robot1::MoveTo10()"},
				new int[] {1});
		operatorSpin7.addConstraint(durationSpin, 0, 0);
		rd.addOperator(operatorSpin10);
		
		SimpleOperator operatorSpin11 = new SimpleOperator("Robot1::Spin11()",
				new AllenIntervalConstraint[] {SpinAfterMoveToWP},
				new String[] {"Robot1::MoveTo11()"},
				new int[] {1});
		operatorSpin11.addConstraint(durationSpin, 0, 0);
		rd.addOperator(operatorSpin11);
		
		SimpleOperator operatorSpin12 = new SimpleOperator("Robot1::Spin12()",
				new AllenIntervalConstraint[] {SpinAfterMoveToWP},
				new String[] {"Robot1::MoveTo12()"},
				new int[] {1});
		operatorSpin12.addConstraint(durationSpin, 0, 0);
		rd.addOperator(operatorSpin12);
		
///////////////////////////////////////////////////////////////////////////////////////

		SimpleOperator operatorMove1 = new SimpleOperator("Robot1::MoveTo1()",
				new AllenIntervalConstraint[] {moveToDuringCharged},
				new String[] {"Battery::Charged5()"},
				new int[] {1});
		operatorMove1.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMove1);


		SimpleOperator operatorMove2 = new SimpleOperator("Robot1::MoveTo2()",
				new AllenIntervalConstraint[] {moveToDuringCharged},
				new String[] {"Battery::Charged2()"},
				new int[] {1});
		operatorMove2.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMove2);

		SimpleOperator operatorMove3 = new SimpleOperator("Robot1::MoveTo3()",
				new AllenIntervalConstraint[] {moveToDuringCharged},
				new String[] {"Battery::Charged1()"},
				new int[] {1});
		operatorMove3.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMove3);

		SimpleOperator operatorMove4 = new SimpleOperator("Robot1::MoveTo4()",
				new AllenIntervalConstraint[] {moveToDuringCharged},
				new String[] {"Battery::Charged2()"},
				new int[] {1});
		operatorMove4.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMove4);
		
		SimpleOperator operatorMove5 = new SimpleOperator("Robot1::MoveTo5()",
				new AllenIntervalConstraint[] {moveToDuringCharged},
				new String[] {"Battery::Charged2()"},
				new int[] {1});
		operatorMove5.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMove5);

		SimpleOperator operatorMove6 = new SimpleOperator("Robot1::MoveTo6()",
				new AllenIntervalConstraint[] {moveToDuringCharged},
				new String[] {"Battery::Charged3()"},
				new int[] {1});
		operatorMove6.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMove6);
		
		SimpleOperator operatorMove7 = new SimpleOperator("Robot1::MoveTo7()",
				new AllenIntervalConstraint[] {moveToDuringCharged},
				new String[] {"Battery::Charged4()"},
				new int[] {1});
		operatorMove7.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMove7);

		SimpleOperator operatorMove8 = new SimpleOperator("Robot1::MoveTo8()",
				new AllenIntervalConstraint[] {moveToDuringCharged},
				new String[] {"Battery::Charged4()"},
				new int[] {1});
		operatorMove8.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMove8);
		
		SimpleOperator operatorMove9 = new SimpleOperator("Robot1::MoveTo9()",
				new AllenIntervalConstraint[] {moveToDuringCharged},
				new String[] {"Battery::Charged5()"},
				new int[] {1});
		operatorMove9.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMove9);


		SimpleOperator operatorMove10 = new SimpleOperator("Robot1::MoveTo10()",
				new AllenIntervalConstraint[] {moveToDuringCharged},
				new String[] {"Battery::Charged5()"},
				new int[] {1});
		operatorMove10.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMove10);
		
		SimpleOperator operatorMove11 = new SimpleOperator("Robot1::MoveTo11()",
				new AllenIntervalConstraint[] {moveToDuringCharged},
				new String[] {"Battery::Charged1()"},
				new int[] {1});
		operatorMove11.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMove11);


		SimpleOperator operatorMove12 = new SimpleOperator("Robot1::MoveTo12()",
				new AllenIntervalConstraint[] {moveToDuringCharged},
				new String[] {"Battery::Charged1()"},
				new int[] {1});
		operatorMove12.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMove12);
		
/////////////////////////////////////////////////////////////////////////////////////////////
		
		SimpleOperator operatorC1 = new SimpleOperator("Battery::Charged1()",
				new AllenIntervalConstraint[] {ChargedAfterCharging},
				new String[] {"Robot1::Wait1()"},
				null);
		operatorC1.addConstraint(durationCharged, 0, 0);
		rd.addOperator(operatorC1);
		
		SimpleOperator operatorC2 = new SimpleOperator("Battery::Charged2()",
				new AllenIntervalConstraint[] {ChargedAfterCharging},
				new String[] {"Robot1::Wait2()"},
				null);
		operatorC2.addConstraint(durationCharged, 0, 0);
		rd.addOperator(operatorC2);
		
		SimpleOperator operatorC3 = new SimpleOperator("Battery::Charged3()",
				new AllenIntervalConstraint[] {ChargedAfterCharging},
				new String[] {"Robot1::Wait3()"},
				null);
		operatorC3.addConstraint(durationCharged, 0, 0);
		rd.addOperator(operatorC3);
		
		SimpleOperator operatorC4 = new SimpleOperator("Battery::Charged4()",
				new AllenIntervalConstraint[] {ChargedAfterCharging},
				new String[] {"Robot1::Wait4()"},
				null);
		operatorC4.addConstraint(durationCharged, 0, 0);
		rd.addOperator(operatorC4);
		
		SimpleOperator operatorC5 = new SimpleOperator("Battery::Charged5()",
				new AllenIntervalConstraint[] {ChargedAfterCharging},
				new String[] {"Robot1::Wait5()"},
				null);
		operatorC5.addConstraint(durationCharged, 0, 0);
		rd.addOperator(operatorC5);
		
		
		SimpleOperator operatorW1 = new SimpleOperator("Robot1::Wait1()",
				new AllenIntervalConstraint[] {ChargingAfterMoveToC},
				new String[] {"Robot1::MoveToC1()"},
				new int[] {1});
		operatorW1.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorW1);
		
		SimpleOperator operatorW2 = new SimpleOperator("Robot1::Wait2()",
				new AllenIntervalConstraint[] {ChargingAfterMoveToC},
				new String[] {"Robot1::MoveToC2()"},
				new int[] {1});
		operatorW2.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorW2);
		
		
		SimpleOperator operatorW3 = new SimpleOperator("Robot1::Wait3()",
				new AllenIntervalConstraint[] {ChargingAfterMoveToC},
				new String[] {"Robot1::MoveToC3()"},
				new int[] {1});
		operatorW3.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorW3);
		
		SimpleOperator operatorW4 = new SimpleOperator("Robot1::Wait4()",
				new AllenIntervalConstraint[] {ChargingAfterMoveToC},
				new String[] {"Robot1::MoveToC4()"},
				new int[] {1});
		operatorW4.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorW4);
		
		
		SimpleOperator operatorW5 = new SimpleOperator("Robot1::Wait5()",
				new AllenIntervalConstraint[] {ChargingAfterMoveToC},
				new String[] {"Robot1::MoveToC5()"},
				new int[] {1});
		operatorW5.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorW5);
		

		SimpleOperator operatorMoveC1 = new SimpleOperator("Robot1::MoveToC1()",
				null,
				null,
				new int[] {1});
		operatorMoveC1.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMoveC1);
		
		SimpleOperator operatorMoveC2 = new SimpleOperator("Robot1::MoveToC2()",
				null,
				null,
				new int[] {1});
		operatorMoveC2.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMoveC2);
		
		SimpleOperator operatorMoveC3 = new SimpleOperator("Robot1::MoveToC3()",
				null,
				null,
				new int[] {1});
		operatorMoveC3.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMoveC3);
		
		SimpleOperator operatorMoveC4 = new SimpleOperator("Robot1::MoveToC4()",
				null,
				null,
				new int[] {1});
		operatorMoveC4.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMoveC4);
		
		SimpleOperator operatorMoveC5 = new SimpleOperator("Robot1::MoveToC5()",
				null,
				null,
				new int[] {1});
		operatorMoveC5.addConstraint(durationMoveTo, 0, 0);
		rd.addOperator(operatorMoveC5);
		
		
		

		//This adds the domain as a meta-constraint of the SimplePlanner
		planner.addMetaConstraint(rd);
		//... and we also add all its resources as separate meta-constraints
		for (Schedulable sch : rd.getSchedulingMetaConstraints()) planner.addMetaConstraint(sch);
		
		
		
		
		
		// INITIAL AND GOAL STATE DEFS
		SymbolicVariableActivity one = (SymbolicVariableActivity)groundSolver.createVariable("Robot1");
		one.setSymbolicDomain("Spin1()");
		// ... this is a goal (i.e., an activity to justify through the meta-constraint)
		one.setMarking(markings.UNJUSTIFIED);
		//.. let's also give it a minimum duration
		AllenIntervalConstraint durationOne = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3,APSPSolver.INF));
		durationOne.setFrom(one);
		durationOne.setTo(one);

		SymbolicVariableActivity two = (SymbolicVariableActivity)groundSolver.createVariable("Robot1");
		two.setSymbolicDomain("Spin2()");
		two.setMarking(markings.UNJUSTIFIED);
		AllenIntervalConstraint durationTwo = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3,APSPSolver.INF));
		durationTwo.setFrom(two);
		durationTwo.setTo(two);
		
		SymbolicVariableActivity three = (SymbolicVariableActivity)groundSolver.createVariable("Robot1");
		three.setSymbolicDomain("Spin3()");
		three.setMarking(markings.UNJUSTIFIED);
		AllenIntervalConstraint durationThree = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3,APSPSolver.INF));
		durationThree.setFrom(three);
		durationThree.setTo(three);

		SymbolicVariableActivity four = (SymbolicVariableActivity)groundSolver.createVariable("Robot1");
		four.setSymbolicDomain("Spin4()");
		four.setMarking(markings.UNJUSTIFIED);
		AllenIntervalConstraint durationFour = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3,APSPSolver.INF));
		durationFour.setFrom(four);
		durationFour.setTo(four);
		
		SymbolicVariableActivity five = (SymbolicVariableActivity)groundSolver.createVariable("Robot1");
		five.setSymbolicDomain("Spin5()");
		five.setMarking(markings.UNJUSTIFIED);
		AllenIntervalConstraint durationFive = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3,APSPSolver.INF));
		durationFive.setFrom(five);
		durationFive.setTo(five);

		SymbolicVariableActivity six = (SymbolicVariableActivity)groundSolver.createVariable("Robot1");
		six.setSymbolicDomain("Spin6()");
		six.setMarking(markings.UNJUSTIFIED);
		AllenIntervalConstraint durationSix = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3,APSPSolver.INF));
		durationSix.setFrom(six);
		durationSix.setTo(six);
		
		SymbolicVariableActivity seven = (SymbolicVariableActivity)groundSolver.createVariable("Robot1");
		seven.setSymbolicDomain("Spin7()");
		seven.setMarking(markings.UNJUSTIFIED);
		AllenIntervalConstraint durationSeven = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3,APSPSolver.INF));
		durationSeven.setFrom(seven);
		durationSeven.setTo(seven);

//		SymbolicVariableActivity eight = (SymbolicVariableActivity)groundSolver.createVariable("Robot1");
//		eight.setSymbolicDomain("Spin8()");
//		eight.setMarking(markings.UNJUSTIFIED);
//		AllenIntervalConstraint durationEight = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3,APSPSolver.INF));
//		durationEight.setFrom(eight);
//		durationEight.setTo(eight);
//		
//		SymbolicVariableActivity nine = (SymbolicVariableActivity)groundSolver.createVariable("Robot1");
//		nine.setSymbolicDomain("Spin9()");
//		nine.setMarking(markings.UNJUSTIFIED);
//		AllenIntervalConstraint durationNine = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3,APSPSolver.INF));
//		durationNine.setFrom(nine);
//		durationNine.setTo(nine);
//
//		SymbolicVariableActivity ten = (SymbolicVariableActivity)groundSolver.createVariable("Robot1");
//		ten.setSymbolicDomain("Spin10()");
//		ten.setMarking(markings.UNJUSTIFIED);
//		AllenIntervalConstraint durationTen = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3,APSPSolver.INF));
//		durationTen.setFrom(ten);
//		durationTen.setTo(ten);
//		
//		SymbolicVariableActivity eleven = (SymbolicVariableActivity)groundSolver.createVariable("Robot1");
//		eleven.setSymbolicDomain("Spin11()");
//		eleven.setMarking(markings.UNJUSTIFIED);
//		AllenIntervalConstraint durationEleven = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3,APSPSolver.INF));
//		durationEleven.setFrom(eleven);
//		durationEleven.setTo(eleven);
//
//		SymbolicVariableActivity twelve = (SymbolicVariableActivity)groundSolver.createVariable("Robot1");
//		twelve.setSymbolicDomain("Spin12()");
//		twelve.setMarking(markings.UNJUSTIFIED);
//		AllenIntervalConstraint durationTwelve = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3,APSPSolver.INF));
//		durationTwelve.setFrom(twelve);
//		durationTwelve.setTo(twelve);

		groundSolver.addConstraints(new Constraint[] {durationOne, durationTwo, durationThree, durationFour, durationFive, durationSix, durationSeven}); //, durationEight, durationNine, durationTen, durationEleven, durationTwelve


//		SymbolicVariableActivity three = (SymbolicVariableActivity)groundSolver.createVariable("Robot1");
//		three.setSymbolicDomain("MoveToC1()");
//		three.setMarking(markings.UNJUSTIFIED);
//		AllenIntervalConstraint durationThree = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3,APSPSolver.INF));
//		durationThree.setFrom(three);
//		durationThree.setTo(three);

//		groundSolver.addConstraints(new Constraint[] {durationOne, durationTwo, durationThree});
		
		// We can also specify that goals should be related in time somehow...		
//		AllenIntervalConstraint after1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.After, AllenIntervalConstraint.Type.After.getDefaultBounds());
//		after1.setFrom(two);
//		after1.setTo(three);
//		groundSolver.addConstraint(after1);
//		
//		AllenIntervalConstraint after2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.After, AllenIntervalConstraint.Type.After.getDefaultBounds());
//		after2.setFrom(six);
//		after2.setTo(five);
//		groundSolver.addConstraint(after2);
//		
//		AllenIntervalConstraint after3 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.After, AllenIntervalConstraint.Type.After.getDefaultBounds());
//		after3.setFrom(seven);
//		after3.setTo(six);
//		groundSolver.addConstraint(after3);
//		
//		AllenIntervalConstraint after4 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.After, AllenIntervalConstraint.Type.After.getDefaultBounds());
//		after4.setFrom(one);
//		after4.setTo(eight);
//		groundSolver.addConstraint(after4);
		
//		AllenIntervalConstraint after5 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.After, AllenIntervalConstraint.Type.After.getDefaultBounds());
//		after5.setFrom(ten);
//		after5.setTo(one);
//		groundSolver.addConstraint(after5);


		TimelinePublisher tp = new TimelinePublisher(groundSolver.getConstraintNetwork(), "Robot1", "Battery");
		TimelineVisualizer viz = new TimelineVisualizer(tp);
		tp.publish(false, false);
		//the following call is marked as "skippable" and will most likely be skipped because the previous call has not finished rendering...
		tp.publish(false, true);
		
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
		
		BufferedWriter writer = new BufferedWriter(new FileWriter("./output/OutputTurtle.txt"));
		
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
