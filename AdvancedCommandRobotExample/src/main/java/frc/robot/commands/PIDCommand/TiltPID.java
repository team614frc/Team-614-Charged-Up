// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PIDCommand;

import java.lang.reflect.Array;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TiltPID extends PIDCommand {
  /** Creates a new TiltPID. */
  static double tiltSetpoint = 0;
  public TiltPID() {
    super(
        // The controller that the command will use
        new PIDController(Constants.kP, Constants.kI, Constants.kD),
        // This should return the measurement
        RobotContainer.tiltSubsystem::getHeight,
        // This should return the setpoint (can also be a constant)
        tiltSetpoint = (RobotContainer.tiltSubsystem.getHeight() > Constants.TILT_DOWN_SETPOINT) ?
        Constants.TILT_DOWN_SETPOINT : Constants.TILT_UP_SETPOINT,
        // This uses the output
        RobotContainer.tiltSubsystem::set);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.tiltSubsystem);
    // Configure additional PID options by calling `getController` here.
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
