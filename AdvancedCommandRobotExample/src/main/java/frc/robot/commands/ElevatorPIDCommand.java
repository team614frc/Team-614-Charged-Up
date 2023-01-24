// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorPIDCommand extends PIDCommand {
  /** Creates a new ElevatorPIDCommand. */
  public static ElevatorSubsystem elevator;
  double elevatorSetpoint;
  //Makes 3 setpoints for elevator subsystem to check if at certain height
  public ElevatorPIDCommand(double elevatorSetpoint) {
    super(
        // The controller that the command will use
        new PIDController(Constants.kP, Constants.kI, Constants.kD),
        // This should return the measurement
        () -> 0,
        // This should return the setpoint (can also be a constant)
        () -> elevatorSetpoint,
        // This uses the output
        output -> {
          // Use the output here
          elevator.set(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
