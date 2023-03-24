package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerLoadEventMapCommand;
import frc.robot.commands.getEventMap;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCube;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCubeAuto;

public class ScoreGrabAuto extends SequentialCommandGroup {
  public ScoreGrabAuto() {
    addCommands(
      new ScoreMidCubeAuto(),
        new PathPlannerLoadEventMapCommand("TopRedScoreGrab", getEventMap.ScoreGrab()).withTimeout(15));
  }
}
