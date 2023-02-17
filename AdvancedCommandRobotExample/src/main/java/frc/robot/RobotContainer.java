// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



//import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.SetLEDColorCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.TiltSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autonomous.TimedBasedAuto.ChargeStationNotEngaged;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.Autonomous.TimedBasedAuto.ScoreMobilityChargeStationEngaged;
import frc.robot.commands.Autonomous.TimedBasedAuto.ScoreMobilityChargeStationNotEngaged;
import frc.robot.commands.Autonomous.TimedBasedAuto.test;
import frc.robot.commands.PIDCommand.ElevatorPIDCommand;
import frc.robot.commands.PIDCommand.TiltPID;
import frc.robot.commands.SimpleCommands.Intake;

public class RobotContainer {
  // Encoder
  // public static Encoder elevatorEncoder = new Encoder(0, 1);
  // Subsystem Initalization
  public static DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  public static Timer autoTimer;
  public static CommandXboxController m_CommandXboxController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  public static Manipulator manipulator = new Manipulator();
  public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public static TiltSubsystem tiltSubsystem = new TiltSubsystem();
  public static LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  public static LEDSubsystem ledSubsystem = new LEDSubsystem();
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final Command chargedStationNotEngaged = new ChargeStationNotEngaged();
  private final Command scoreMobilityChargeStationEngaged = new ScoreMobilityChargeStationEngaged();
  private final Command scoreMobilityChargeStationNotEngaged = new ScoreMobilityChargeStationNotEngaged();
  private final Command test = new test();

  // Timed Auto Initaliztion

  public RobotContainer() {
    configureBindings();
    // Calling Arcade Drive Comment
    driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());
    //manipulator.setDefaultCommand(new ManipulatorDefaultCommand(RobotContainer.manipulator));

    // Add commands to the autonomous command chooser
    m_chooser.addOption("Charge_Station_Not_Engaged", chargedStationNotEngaged);
    m_chooser.addOption("Score_Mobility_Charge_Station_Engaged", scoreMobilityChargeStationEngaged);
    m_chooser.addOption("Score_Mobility_Charge_Station_Not_Engaged", scoreMobilityChargeStationNotEngaged);
    m_chooser.setDefaultOption("test", test);

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);

    SmartDashboard.putNumber("Drivetrain Encoder Position", driveTrainSubsystem.getPosition());
  }


  private void configureBindings() {
    m_CommandXboxController.button(Constants.LEFT_BUMPER).whileTrue(new Intake(Constants.MANIPULATOR_SETPOINT));
    m_CommandXboxController.button(Constants.LEFT_STICK_PRESS).whileTrue(new Intake(Constants.MANIPULATOR_SETPOINT2));
    m_CommandXboxController.button(Constants.Y_BUTTON).whileTrue(new ElevatorPIDCommand(Constants.ELEVATOR_SETPOINT));
    m_CommandXboxController.button(Constants.X_BUTTON).whileTrue(new ElevatorPIDCommand(Constants.ELEVATOR_SETPOINT2));
    m_CommandXboxController.button(Constants.RIGHT_BUMPER).whileTrue(new TiltPID());
    m_CommandXboxController.button(Constants.START_BUTTON).toggleOnTrue(new SetLEDColorCommand(0)); //Sets LED's to purple
    m_CommandXboxController.button(Constants.BACK_BUTTON).toggleOnTrue(new SetLEDColorCommand(1)); //Sets LED's to yellow
    //m_CommandXboxController.button(Constants.RIGHT_STICK_PRESS).whileTrue(new TiltPID());
    
  }

  public Command getAutonomousCommand() {
    // Gets the selected autonomous command

    return m_chooser.getSelected();
  }
}
