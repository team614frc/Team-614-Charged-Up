// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.TimedBasedAuto.Timed.DriveDirection;
import frc.robot.commands.Autonomous.TimedBasedAuto.Timed.TimedExtend;
import frc.robot.commands.Autonomous.TimedBasedAuto.Timed.TimedIntake;
import frc.robot.commands.Autonomous.TimedBasedAuto.Timed.TimedTilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreMobilityChargeStationNotEngaged extends SequentialCommandGroup {
  /** Creates a new ScoreMobilityChargeStationNotEngaged. */
  public ScoreMobilityChargeStationNotEngaged() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new TimedTilt(0.5, 1),
     new TimedExtend(0.5, 1),
      new TimedIntake(0.5, 5),
       new TimedTilt(-0.5, 1),
       new DriveDirection(0.5, 0, 2),
        new DriveDirection(-0.5, 0, 1));
  }
}
