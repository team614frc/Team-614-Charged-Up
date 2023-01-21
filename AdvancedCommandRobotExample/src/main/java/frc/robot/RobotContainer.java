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
import frc.robot.commands.Autonomous.AutoChargeStationNotEngaged;
import frc.robot.commands.Autonomous.AutoScoreChargeStation;
import frc.robot.commands.Autonomous.TimedBasedAuto.TimedAuto;
import frc.robot.commands.Autonomous.TimedBasedAuto.TimedAutoScoreMobilityPad;
import frc.robot.subsystems.DriveTrainSubsystem;

public class RobotContainer {

  // Subsystem Initalization
  public static DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  public static Timer autoTimer;

  // Contollers Initalization
  public static XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);

  // A simple auto routine that drives forward a specified distance, and then stops.
  private final Command TimedAuto = new TimedAuto();
  // A complex auto routine that drives forward, scores, gets mobility, and gets pad
  private final Command ScoreMobilityChargeStation = new TimedAutoScoreMobilityPad();
  // A complex auto routine that scores, and charge station - not engaged
  private final Command ScoreChargeStation = new AutoScoreChargeStation();
  // A complex auto routine that charge station - not engaged
  private final Command ChargeStationNotEngaged = new AutoChargeStationNotEngaged();

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    System.out.println("in robot container");
    configureBindings();
    //Calling Arcade Drive Comment 
    driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());
    
    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Simple Auto", ScoreChargeStation);
    m_chooser.addOption("Complex Auto", ScoreMobilityChargeStation);
    m_chooser.addOption("Complex Auto2", ChargeStationNotEngaged);

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
