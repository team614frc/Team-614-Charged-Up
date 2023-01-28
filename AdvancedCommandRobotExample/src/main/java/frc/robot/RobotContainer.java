// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Autonomous.TimedBasedAuto.ChargeStationNotEngaged;
import frc.robot.commands.Autonomous.TimedBasedAuto.ScoreMobilityChargeStationNotEngaged;
import frc.robot.subsystems.DriveTrainSubsystem;

public class RobotContainer {

  // Subsystem Initalization
  public static DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  public static Timer autoTimer;

  // Contollers Initalization
  public static XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);

  // A simple auto routine that drives forward a specified distance, and then stops.
  private final Command ChargeStationNotEngaged = new ChargeStationNotEngaged();
  // A complex auto that scores, gets mobility, and gets charge station not engaged.
  private final Command ScoreMobilityChargeStationNotEngaged = new ScoreMobilityChargeStationNotEngaged();

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    //Calling Arcade Drive Comment 
    driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());
    
    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("ChargeStationNotEngaged", ChargeStationNotEngaged);
    m_chooser.addOption("ScoreMobilityChargeStationNotEngaged", ScoreMobilityChargeStationNotEngaged);

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
