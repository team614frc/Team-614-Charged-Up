// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PIDCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TiltPID extends PIDCommand {
  /** Creates a new TiltPID. */
  static double tiltSetpoint;

  public TiltPID() { 
    super(
        // The controller that the command will use
        new PIDController(Constants.P_kP, Constants.P_kI, Constants.P_kD),
        // This should return the measurement
        RobotContainer.tiltSubsystem::getHeight,
        /*A ternary operator, if tiltSetpoint is currently 0 (default), then set setpoint to up setpoint,
        else if current tiltSetpoint is up setpoint, set setpoint to 0.
        */
        tiltSetpoint = (RobotContainer.tiltSubsystem.getHeight() > Constants.TILT_DOWN_SETPOINT) ? Constants.TILT_DOWN_SETPOINT 
        : (RobotContainer.tiltSubsystem.getHeight() == Constants.TILT_DOWN_SETPOINT) ? Constants.TILT_UP_SETPOINT : tiltSetpoint,
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
