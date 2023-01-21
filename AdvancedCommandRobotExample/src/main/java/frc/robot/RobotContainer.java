// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.IntakeWheels;
import frc.robot.commands.Autonomous.TimedBasedAuto.TimedAuto;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.XboxController.Button;

public class RobotContainer {

  // Subsystem Initalization
  public static DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  public static Timer autoTimer;
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // Contollers Initalization
 //public static XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
  public static CommandXboxController m_CommandXboxController = new CommandXboxController(0);
  //final Trigger leftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);

  IntakeWheels wheels = new IntakeWheels();

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
  //driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());
    
    //leftBumper.onTrue(new IntakeWheels()).onFalse(m_TimedAutoCommand);
    //driverController.a().onTrue(wheels.test());
    //m_CommandXboxController.leftBumper().onTrue(new IntakeWheels());
    m_CommandXboxController.button(1).onTrue(new IntakeWheels());
   
   }

  public Command getAutonomousCommand() {
    return m_TimedAutoCommand;// Commands.print("No autonomous command configured");
  }
}
