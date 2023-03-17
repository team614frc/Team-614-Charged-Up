package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

public class PathPlannerLoadEventMapCommand extends InstantCommand {
  private final String filename;
  private final boolean resetOdomtry;
  private final HashMap<String, Command> eventMap;

  public PathPlannerLoadEventMapCommand(String filename, boolean resetOdomtry, HashMap<String, Command> eventMap) {
    this.filename = filename;
    this.resetOdomtry = resetOdomtry;
    this.eventMap = eventMap;
  }

  @Override
  public void initialize() {
    addRequirements(RobotContainer.driveTrainSubsystem);
    PathConstraints constraints = new PathConstraints(2, 2);
    PathPlannerTrajectory examplePath = PathPlanner.loadPath(filename, constraints);

    FollowPathWithEvents command = new FollowPathWithEvents(
        new PathPlannerLoadPathCommand(filename, resetOdomtry),
        examplePath.getMarkers(),
        eventMap);

    if (resetOdomtry) {
      new SequentialCommandGroup(
          new InstantCommand(
              () -> RobotContainer.driveTrainSubsystem.resetOdometry(examplePath.getInitialPose())),
          command).schedule();
    } else {
      command.schedule();
    }
  }
}