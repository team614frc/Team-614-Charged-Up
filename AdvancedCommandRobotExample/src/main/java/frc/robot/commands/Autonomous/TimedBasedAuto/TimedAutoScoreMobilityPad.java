// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TimedAutoScoreMobilityPad extends CommandBase {
  /** Creates a new TimedAuto. */
  Timer arcadeDriveTimer = null; 

  public TimedAutoScoreMobilityPad() {
    // Use addRequirements() here to declare subsystem dependencies.
    // DriveTrain Subsystem required for arcade drive 
    addRequirements(RobotContainer.driveTrainSubsystem);

    arcadeDriveTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arcadeDriveTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  //   System.out.println("Stage One, line up to score");
  //   if (arcadeDriveTimer.get() <= Constants.RUN_INITAL_AUTO) { 
  //     RobotContainer.driveTrainSubsystem.arcadeDrive(1, 0);
  //   }
  //   System.out.println("Stage Two, score");
  //   if (arcadeDriveTimer.get() <= Constants.AUTO_STAGE_2) { 
  //   RobotContainer.driveTrainSubsystem.arcadeDrive(0, 0);
  //   }
  //   System.out.println("Stage Three, mobility");
  //   if (arcadeDriveTimer.get() <= Constants.AUTO_STAGE_3) { 
  //   RobotContainer.driveTrainSubsystem.arcadeDrive(-1, 0);
  //   }
  //   System.out.println("Stage Four, charge station");
  //   if (arcadeDriveTimer.get() <= Constants.AUTO_STAGE_4) { 
  //   RobotContainer.driveTrainSubsystem.arcadeDrive(1, 0);
  // }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arcadeDriveTimer.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
