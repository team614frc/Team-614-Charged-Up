// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.PathPlannerCommands.PathPlannerLoadPathCommand;
import frc.robot.Commands.SequentialParallelCommands.ScoreHighCubeAuto;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto13 extends SequentialCommandGroup {
  /** Creates a new Auto13. */
  public Auto13() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreHighCubeAuto(),
      new PathPlannerLoadPathCommand("pathplanner/generatedJSON/BTCMM.wpilib.json", true)
    );
  }
}