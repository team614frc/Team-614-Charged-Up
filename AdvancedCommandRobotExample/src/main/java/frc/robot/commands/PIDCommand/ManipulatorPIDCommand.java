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
public class ManipulatorPIDCommand extends PIDCommand {
  /** Creates a new ManipulatorPIDCommand. */
  public ManipulatorPIDCommand(double manipulatorSetpoint) {
    super(
        // The controller that the command will use
        new PIDController(Constants.P_kP, Constants.P_kI, Constants.P_kD),
        // Returns current intake speed
        RobotContainer.manipulator::getSpeed,
        // Could be used to hard code setpoint, but code requires two button presses that dictate setpoint
        manipulatorSetpoint,
        // This uses the output
        
          RobotContainer.manipulator::set);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.manipulator);
  }
  // Returns true when the command should end.

  public void end(boolean interrupted) {
    RobotContainer.manipulator.set(Constants.MOTOR_REST_BACK);
  }
  @Override
  public boolean isFinished() {
    return false;
  }


}
