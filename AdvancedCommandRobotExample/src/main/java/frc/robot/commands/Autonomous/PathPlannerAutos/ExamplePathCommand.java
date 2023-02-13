package frc.robot.commands.Autonomous.PathPlannerAutos;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PIDCommand.DrivePositionPIDCommand;

public class ExamplePathCommand extends SequentialCommandGroup {
  public ExamplePathCommand() {
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    eventMap.put("intakeDown", new DrivePositionPIDCommand(1));
    addCommands(new FollowPathWithEvents(new PathPlannerAutos(examplePath), examplePath.getMarkers(), eventMap));
  }
}
