// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Encoder;

//import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ElevatorPIDCommand;
import frc.robot.commands.Extend;
import frc.robot.commands.Intake;
import frc.robot.commands.ManipulatorPIDCommand;
import frc.robot.commands.Tilt;
import frc.robot.commands.Autonomous.TimedBasedAuto.TimedAuto;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Manipulator;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.XboxController.Button;


public class RobotContainer {
  //Encoder 
  //public static Encoder elevatorEncoder = new Encoder(0, 1);
  // Subsystem Initalization
  public static DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  public static Timer autoTimer;

  public static Manipulator manipulator = new Manipulator();
  public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();



  // Timed Auto Initaliztion
  public static TimedAuto m_TimedAutoCommand = new TimedAuto();

  public RobotContainer() {
    System.out.println("in robot container");

    // leftBumper
    // Calling Arcade Drive Comment
    driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());

    configureBindings();
  }

  private void configureBindings() {

   }

  public Command getAutonomousCommand() {
    return m_TimedAutoCommand;// Commands.print("No autonomous command configured");

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
      //driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());
    
    //leftBumper.onTrue(new IntakeWheels()).onFalse(m_TimedAutoCommand);
    //driverController.a().onTrue(wheels.test());
    //m_CommandXboxController.leftBumper().onTrue(new IntakeWheels());
    m_CommandXboxController.button(5).whileTrue(new ManipulatorPIDCommand(Constants.MANIPULATOR_SETPOINT));
    m_CommandXboxController.button(9).whileTrue(new ManipulatorPIDCommand(Constants.MANIPULATOR_SETPOINT2));
    m_CommandXboxController.button(4).whileTrue(new ElevatorPIDCommand(Constants.MANIPULATOR_SETPOINT));
    m_CommandXboxController.button(3).whileTrue(new ElevatorPIDCommand(Constants.MANIPULATOR_SETPOINT2));
    m_CommandXboxController.button(7).whileTrue(new Tilt(Constants.TILT_UP_SPEED));
    m_CommandXboxController.button(8).whileTrue(new Tilt(Constants.TILT_DOWN_SPEED));
  }

  public Command getAutonomousCommand() {
    // Gets the selected autonomous command
    return m_chooser.getSelected();

  }
}
