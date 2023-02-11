// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DrivePositionPIDCommand extends PIDCommand {
  /** Creates a new PositionPID. */

  double driveSetpoint;

  @Override
  public void initialize() {
    RobotContainer.driveTrainSubsystem.resetEncoderValues();
  }
  public DrivePositionPIDCommand(double driveSetpoint) {
    super(
        // The controller that the command will use
        new PIDController(Constants.P_kP, Constants.P_kI, Constants.P_kD),
        // This should return the measurement
        RobotContainer.driveTrainSubsystem::getPosition,
        // This should return the setpoint (can also be a constant)
        driveSetpoint,
        // This uses the output
        RobotContainer.driveTrainSubsystem::setSpeed);
        SmartDashboard.putNumber("Position setpoint value", getController().getSetpoint());
        // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrainSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(1);
  }

  // Returns true when the command should end.
 

  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("Position setpoint value", getController().getSetpoint());
    return getController().atSetpoint();
  }
}
