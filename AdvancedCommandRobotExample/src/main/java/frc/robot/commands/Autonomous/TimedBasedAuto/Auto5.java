package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerLoadPathCommand;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCube;

public class Auto5 extends SequentialCommandGroup {
  public Auto5() {
    addCommands(
      new ScoreMidCube(),
      new PathPlannerLoadPathCommand("pathplanner/generatedJSON/RTCMCS.wpilib.json", true)
    );
  }
}
