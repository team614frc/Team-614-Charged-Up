// package frc.robot.commands;

// import java.util.HashMap;
// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.FollowPathWithEvents;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
// import frc.robot.commands.SequentialParallelCommands.LoadStation;

// public class PathPlannerEventMapLoader extends InstantCommand {

//   public PathPlannerEventMapLoader() {
//   }

//   @Override
//   public void initialize() {
// // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));

// // This is just an example event map. It would be better to have a constant, global event map
// // in your code that will be used by all path following commands.
// HashMap<String, Command> eventMap = new HashMap<>();
// // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
// eventMap.put("intakeDown", new IntakeDown());

// FollowPathWithEvents command = new FollowPathWithEvents(
//     getPathFollowingCommand(examplePath),
//     examplePath.getMarkers(),
//     eventMap
// );
//   }
// }
