// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotatePIDCommand extends PIDCommand {
  /** Creates a new RotatePIDCommand. */

  public RotatePIDCommand(double degrees) {
    super(
        // The controller that the command will use
        new PIDController(Constants.P_kP, Constants.P_kI, Constants.P_kD),
        // This should return the measurement
        RobotContainer.driveTrainSubsystem.navx::getAngle,
        // This should return the setpoint (can also be a constant)
        degrees,
        // This uses the output
        output -> {
          // if statement to ensure motors do not output too much or too little power to
          // reach setpoint
          RobotContainer.driveTrainSubsystem.arcadeDrive(0, -output - (0.14402 / 12));
          SmartDashboard.putNumber("Output:", output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrainSubsystem);
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("Rotate P Value:", Constants.P_kP);
    SmartDashboard.putNumber("Rotate I Value:", Constants.P_kI);
    SmartDashboard.putNumber("Rotate D Value:", Constants.P_kD);
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(1);
  }

  @Override
  public void execute() {
    super.execute();
    SmartDashboard.putNumber("Rotate Gyro setpoint value", getController().getSetpoint());
    SmartDashboard.putNumber("Rotate Gyro current value", RobotContainer.driveTrainSubsystem.navx.getAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

}
