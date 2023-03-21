package frc.robot.commands;

import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class PathPlannerLoadEventMapCommand extends InstantCommand {

  public PathPlannerLoadEventMapCommand() {
  }

  @Override
  public void initialize() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("ScoreGrabBalance", new PathConstraints(2, 2));

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("intake", new PrintCommand("left"));
    eventMap.put("right", new PrintCommand("right"));
    eventMap.put("event", new PrintCommand("end"));

    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
        RobotContainer.driveTrainSubsystem::getPose,
        RobotContainer.driveTrainSubsystem::resetOdometry,
        new RamseteController(),
        Constants.kDriveKinematics,
        RobotContainer.driveTrainSubsystem::DifferentialDriveVolts,
        eventMap,
        RobotContainer.driveTrainSubsystem);

    autoBuilder.fullAuto(pathGroup).schedule();
  }
}
