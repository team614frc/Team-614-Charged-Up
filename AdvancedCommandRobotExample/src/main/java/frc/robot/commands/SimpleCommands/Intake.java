package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends CommandBase {
  public double intakeSpeed;
  // public boolean stopRunning;
  public Intake(double intakespeed) {
    addRequirements(RobotContainer.manipulator);
    intakeSpeed = intakespeed;
  }

  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If left bumper is pressed, turns the motor on in order to take the game piece
    // in, and grip it
    RobotContainer.manipulator.set(intakeSpeed);
    // if ((RobotContainer.manipulator.pdh.getCurrent(Constants.INTAKE_MOTOR) >= Constants.MANIPULATOR_THRESHOLD) && (Timer.getFPGATimestamp() > 5) && !stopRunning) {
    //   stopRunning = true;
    //   SmartDashboard.putBoolean("Picked Up game piece:", true);
    // }
    //   else {
    //     stopRunning = false;
    //     SmartDashboard.putBoolean("Picked Up game piece:", false);
    //   }
    //   SmartDashboard.putNumber("Current Outputted by PDH:", RobotContainer.manipulator.pdh.getCurrent(Constants.INTAKE_MOTOR));
    }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.manipulator.set(Constants.MOTOR_ZERO_SPEED);
  }

  // gets returned true when the command ends
  @Override
  public boolean isFinished() {
    return false;
  }
}
