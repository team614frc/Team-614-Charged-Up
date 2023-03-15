package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
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
  private PathPlannerTrajectory examplePath;
  private FollowPathWithEvents command;
  private final String filename;
  private final boolean resetOdomtry;
  private final HashMap<String, Command> eventMap;
  private Trajectory trajectory;

  public PathPlannerLoadEventMapCommand(String filename, boolean resetOdomtry, HashMap<String, Command> eventMap) {
    this.filename = filename;
    this.resetOdomtry = resetOdomtry;
    this.eventMap = eventMap;
  }

  @Override
  public void initialize() {
    addRequirements(RobotContainer.driveTrainSubsystem);
    PathConstraints constraints = new PathConstraints(2, 2);
    examplePath = PathPlanner.loadPath(filename, constraints);

    command = new FollowPathWithEvents(
        new PathPlannerLoadPathCommand(filename, resetOdomtry),
        examplePath.getMarkers(),
        eventMap);

    if (resetOdomtry) {
      new SequentialCommandGroup(
          new InstantCommand(
              () -> RobotContainer.driveTrainSubsystem.resetOdometry(trajectory.getInitialPose())),
          command).schedule();
    } else {
      command.schedule();
    }
  }
}