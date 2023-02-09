package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RotateRightPIDCommand extends PIDCommand {

  public RotateRightPIDCommand (double rotationSetpoint) {
    super(

        new PIDController(Constants.V_kP, Constants.V_kI, Constants.V_kD),

        RobotContainer.driveTrainSubsystem::getPosition,

        rotationSetpoint,

        RobotContainer.driveTrainSubsystem::rotateRight);

    addRequirements(RobotContainer.driveTrainSubsystem);

    getController().setTolerance(0.1);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
