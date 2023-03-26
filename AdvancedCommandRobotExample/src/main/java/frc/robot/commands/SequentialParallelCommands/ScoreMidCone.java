package frc.robot.commands.SequentialParallelCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.PIDCommand.TiltPIDCommand;
import frc.robot.commands.SimpleCommands.Extend;
import frc.robot.commands.SimpleCommands.Retract;

public class ScoreMidCone extends SequentialCommandGroup {

  public ScoreMidCone() {
    addCommands(
        new TiltPIDCommand(Constants.TILT_LOAD_STATION_SETPOINT).withTimeout(0.1),
        new Extend().withTimeout(0.6),
        new TiltPIDCommand(Constants.TILT_MID_SCORE_SETPOINT).withTimeout(0.5),
        new Retract().withTimeout(0.5),
        new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT).withTimeout(0.5));
  }
}