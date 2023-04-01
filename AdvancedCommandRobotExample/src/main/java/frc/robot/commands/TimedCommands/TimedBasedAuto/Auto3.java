package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerLoadPathCommand;

public class Auto3 extends SequentialCommandGroup {
  public Auto3() {
    addCommands(
      new PathPlannerLoadPathCommand("pathplanner/generatedJSON/Path3.wpilib.json", true).withTimeout(15)
    );
  }
}
