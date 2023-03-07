package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SetLEDColorCommand extends CommandBase {

  private final int m_color;

  public SetLEDColorCommand(int color) {
    m_color = color;

    addRequirements(RobotContainer.ledSubsystem);
  }

  @Override
  public void execute() {
    if (m_color == 0) {
      RobotContainer.ledSubsystem.setLedColorPurple();
    } else if (m_color == 1) {
      RobotContainer.ledSubsystem.setLedColorYellow();
    } else if (m_color == 2) {
      RobotContainer.ledSubsystem.setLedColorGreen();
    } else if (m_color == 3) {
      RobotContainer.ledSubsystem.setLedColorOrange();
    } else
      RobotContainer.ledSubsystem.setLedColorRainbow();
  }
}
