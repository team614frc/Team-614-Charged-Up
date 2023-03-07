// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.PathPlannerLoadPathCommand;
import frc.robot.commands.PIDCommand.TiltPIDCommand;
import frc.robot.commands.SimpleCommands.Extend;
import frc.robot.commands.SimpleCommands.Intake;
import frc.robot.commands.SimpleCommands.Retract;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto4 extends SequentialCommandGroup {
  /** Creates a new Auto4. */
  public Auto4() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TiltPIDCommand(Constants.TILT_HIGH_CUBE_SETPOINT).withTimeout(0.5),
      new Extend().withTimeout(0.5),
      new Intake(Constants.MANIPULATOR_SPEED_BLEH).withTimeout(0.5),
      new Retract().withTimeout(0.5),
      new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT).withTimeout(0.5),
      new PathPlannerLoadPathCommand("pathplanner/generatedJSON/Path0.wpilib.json", true).withTimeout(15)
    );
  }
}
