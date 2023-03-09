package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerLoadPathCommand;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCube;

public class Auto3 extends SequentialCommandGroup {
  public Auto3() {
    addCommands(
      new ScoreMidCube(),
      new PathPlannerLoadPathCommand("pathplanner/generatedJSON/BTCMM.wpilib.json", true)
    );
  }
}
