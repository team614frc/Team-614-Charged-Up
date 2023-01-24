// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Manipulator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManipulatorPIDCommand extends PIDCommand {
  /** Creates a new ManipulatorPIDCommand. */
  double setpoint;
  public ManipulatorPIDCommand(double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(Constants.kP, Constants.kI, Constants.kD),
        // Returns current intake speed
        () -> RobotContainer.manipulator.getSpeed(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          RobotContainer.manipulator.set(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.manipulator);
    // Configure additional PID options by calling `getController` here.
    getController().getSetpoint();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
