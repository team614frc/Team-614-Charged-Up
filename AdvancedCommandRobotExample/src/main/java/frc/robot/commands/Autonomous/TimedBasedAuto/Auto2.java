package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerLoadPathCommand;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCube;

public class Auto2 extends SequentialCommandGroup {
  public Auto2() {
    addCommands(
      new ScoreMidCube(),
      new PathPlannerLoadPathCommand("pathplanner/generatedJSON/BTCMCS.wpilib.json", true)
    );
  }
}
