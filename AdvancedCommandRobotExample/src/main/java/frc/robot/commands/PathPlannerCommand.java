// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathPlannerCommand extends InstantCommand {
  private final String filename;
  private final boolean resetOdomtry;
  private Trajectory trajectory;
  
  public PathPlannerCommand(String filename, boolean resetOdomtry) {
    this.filename = filename;
    this.resetOdomtry = resetOdomtry;
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  } catch (IOException exception) {
      System.out.println("Unable to read from file " + filename);
      cancel();
  }

  RamseteCommand ramseteCommand = new RamseteCommand(
                trajectory,
                RobotContainer.driveTrainSubsystem::getPose,
                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.ksVolts,
                        Constants.kvVoltSecondsPerMeter,
                        Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics,
                RobotContainer.driveTrainSubsystem::getWheelSpeeds,
                new PIDController(Constants.V_kP, 0, 0),
                new PIDController(Constants.V_kP, 0, 0),
                RobotContainer.driveTrainSubsystem::DifferentialDriveVolts,
                RobotContainer.driveTrainSubsystem);

        if (resetOdomtry) {
            new SequentialCommandGroup(
                    new InstantCommand(
                            () -> RobotContainer.driveTrainSubsystem.resetOdometry(trajectory.getInitialPose())),
                    ramseteCommand).schedule();
        } else {
            ramseteCommand.schedule();
        }
  }
}
