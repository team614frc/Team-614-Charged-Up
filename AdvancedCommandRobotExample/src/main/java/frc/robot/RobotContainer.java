// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DriveBackwards;
import frc.robot.commands.DriveForwards;
import frc.robot.commands.Autonomous.TimedBasedAuto.TimedAuto;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {

  // Subsystem Initalization
  public static DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  public static Timer autoTimer;

  // Contollers Initalization
  public static XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);

  // Timed Auto Initaliztion
  public static TimedAuto m_TimedAutoCommand = new TimedAuto();

  // A simple auto routine that drives forward a specified distance, and then stops.
  private final Command m_simpleAuto = new DriveForwards();

  // A complex auto routine that drives forward, drops a hatch, and then drives backward.
  private final Command m_complexAuto = new DriveBackwards();


  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    System.out.println("in robot container");
    configureBindings();
    //Calling Arcade Drive Comment 
    driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());
    
    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
    m_chooser.addOption("Complex Auto", m_complexAuto);

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);
  }

  private void configureBindings() {

    //driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());
  }

  public Command getAutonomousCommand() {
    // Gets the selected autonomous command
    return m_chooser.getSelected();
  }
}
