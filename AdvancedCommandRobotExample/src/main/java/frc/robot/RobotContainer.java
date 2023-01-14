// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.XboxController;;

public class RobotContainer {

  // Subsystem Initalization
  public static DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();


  // Contollers Initalization
  public static XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);


  public RobotContainer() {
    System.out.println("in robot container");
    configureBindings();
    //driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());
    
  }

  private void configureBindings() {

    driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
