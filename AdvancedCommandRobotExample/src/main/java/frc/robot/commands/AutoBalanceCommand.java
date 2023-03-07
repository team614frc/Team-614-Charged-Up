package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalanceCommand extends CommandBase {
  private double error;
  private double currentAngle;
  private double drivePower;

  public AutoBalanceCommand() {
    addRequirements(RobotContainer.driveTrainSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    this.currentAngle = RobotContainer.driveTrainSubsystem.getPitch();

    error = 0 - currentAngle;
    drivePower = -Math.min(0.015 * error, 1);

    if (Math.abs(drivePower) > 0.4) {
      drivePower = Math.copySign(0.4, drivePower);
    }

    RobotContainer.driveTrainSubsystem.arcadeDrive(drivePower, drivePower);

    SmartDashboard.putNumber("Current Angle: ", currentAngle);
    SmartDashboard.putNumber("Error ", error);
    SmartDashboard.putNumber("Drive Power: ", drivePower);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveTrainSubsystem.arcadeDrive(Constants.MOTOR_ZERO_SPEED, Constants.MOTOR_ZERO_SPEED);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(error) < 1;
  }
}