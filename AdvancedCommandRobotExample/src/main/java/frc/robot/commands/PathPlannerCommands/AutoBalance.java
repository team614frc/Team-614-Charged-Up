// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PathPlannerCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoBalance extends CommandBase {
  private double error;
  private double currentAngle;
  private double lastAngle = 0;
  private double drivePower;
  private long balanceTimeMili = 0;
  private double ForwardMult = 1.5; // must have its own max speed
  private double maxSpeed = 0.5;
  private double diferenceInAngle;
  double stopAngle = 10.0;
  boolean driveBackwards;

  public AutoBalance() {
    addRequirements(RobotContainer.driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("AutoBalanceStopAngle", stopAngle);

    // sets angle to roll: angle the balence beam can rotate.
    this.currentAngle = RobotContainer.driveTrainSubsystem.getRoll();
    if (currentAngle > 12.5) { //6
      RobotContainer.driveTrainSubsystem.arcadeDrive(0.3, 0);
    } else if (currentAngle < -7) { //2
      RobotContainer.driveTrainSubsystem.arcadeDrive(-0.3, 0);
    } else if (currentAngle <= -12.5 && currentAngle >= -7) { //6 & 2
      RobotContainer.driveTrainSubsystem.arcadeDrive(0.0, 0);
      RobotContainer.driveTrainSubsystem.setBreakMode();
    }
    // m_drivetrain.arcadeDrive(0.3, 0);

    // System.out.println("drivePower*FM: "+(drivePower*ForwardMult)+"angle:
    // "+currentAngle+" ForwardMult:"+ForwardMult+" difInAngle: "+diferenceInAngle+"
    // maxSpeed: "+maxSpeed);
    SmartDashboard.putNumber("drivePower*FM", (drivePower * ForwardMult));
    SmartDashboard.putNumber("pitch balance angle", currentAngle);
    SmartDashboard.putNumber("ForwardMult", ForwardMult);
    SmartDashboard.putNumber("difInAngle", diferenceInAngle);
    SmartDashboard.putNumber("AutoBalance maxSpeed", maxSpeed);
    SmartDashboard.putBoolean("balancing", true);

    this.lastAngle = currentAngle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}