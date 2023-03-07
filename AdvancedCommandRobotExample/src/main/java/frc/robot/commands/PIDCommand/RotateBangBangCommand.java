package frc.robot.commands.PIDCommand;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RotateBangBangCommand extends CommandBase {

  double setpoint;
  BangBangController controller;

  public RotateBangBangCommand(double setpoint) {
    this.setpoint = setpoint;
    addRequirements(RobotContainer.driveTrainSubsystem);
    controller = new BangBangController(0.5);
  }

  @Override
  public void initialize() {
    RobotContainer.driveTrainSubsystem.navX.reset();
  }

  @Override
  public void execute() {
    double currentAngle = RobotContainer.driveTrainSubsystem.navX.getAngle();
    double output = controller.calculate(currentAngle, setpoint);
    RobotContainer.driveTrainSubsystem.arcadeDrive(0, output);
    SmartDashboard.putNumber("Rotate setpoint value", setpoint);
    SmartDashboard.putNumber("Rotate current value", currentAngle);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(setpoint - RobotContainer.driveTrainSubsystem.navX.getAngle()) < 1.0; // within a tolerance of 1
                                                                                          // degree
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveTrainSubsystem.arcadeDrive(0, 0);
  }
}
