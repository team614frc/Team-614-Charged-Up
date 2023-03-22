package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.PIDCommand.TiltPIDCommand;
import frc.robot.commands.SequentialParallelCommands.GroundIntake;
import frc.robot.commands.SequentialParallelCommands.LoadStation;
import frc.robot.commands.SequentialParallelCommands.PchooOverCS;
import frc.robot.commands.SequentialParallelCommands.ScoreHighCube;
import frc.robot.commands.SequentialParallelCommands.ScoreHybrid;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCone;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCube;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.TiltSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SimpleCommands.Extend;
import frc.robot.commands.SimpleCommands.MaxTiltUp;
import frc.robot.commands.SimpleCommands.Retract;
import frc.robot.commands.SimpleCommands.MaxTiltDown;
import frc.robot.commands.Autonomous.TimedBasedAuto.Auto1;
import frc.robot.commands.Autonomous.TimedBasedAuto.Auto2;
import frc.robot.commands.Autonomous.TimedBasedAuto.Auto3;
import frc.robot.commands.Autonomous.TimedBasedAuto.Auto4;
import frc.robot.commands.Autonomous.TimedBasedAuto.Auto5;
import frc.robot.commands.Autonomous.TimedBasedAuto.Auto6;
import frc.robot.commands.Autonomous.TimedBasedAuto.TestAuto;
import frc.robot.commands.Autonomous.TimedBasedAuto.BalanceAuto;

public class RobotContainer {
    public static DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
    public static CommandXboxController m_CommandXboxController = new CommandXboxController(
            Constants.DRIVER_CONTROLLER_PORT);
    public static CommandXboxController co_CommandXboxController = new CommandXboxController(
            Constants.CO_DRIVER_CONTROLLER_PORT);

    public static Manipulator manipulator = new Manipulator();
    public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public static TiltSubsystem tiltSubsystem = new TiltSubsystem();
    public static LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    public static LEDSubsystem ledSubsystem = new LEDSubsystem();

    private final Command BBCMMCS = new Auto1();
    private final Command BTCMCS = new Auto2();
    private final Command BTCMM = new Auto3();
    private final Command RBCMMCS = new Auto4();
    private final Command RTCMCS = new Auto5();
    private final Command RTCMM = new Auto6();
    private final Command BalanceAuto = new BalanceAuto();
    private final Command TestAuto = new TestAuto();

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());

        m_chooser.addOption("Blue - Mid Cube Over Charge Station #1", BBCMMCS);
        m_chooser.addOption("Blue - Mid Cube Around Charge Station #2", BTCMCS);
        m_chooser.addOption("Blue - Mid Cube Mobility #3", BTCMM);
        m_chooser.addOption("Red - Mid Cube Over Charge Station #4", RBCMMCS);
        m_chooser.addOption("Red - Mid Cube Around Charge Station #5", RTCMCS);
        m_chooser.addOption("Red - Mid Cube Mobility", RTCMM);
        m_chooser.addOption("Auto Balance", BalanceAuto);
        m_chooser.addOption("--TESTAUTO--", TestAuto);

        SmartDashboard.putData(m_chooser);
    }

    private void configureBindings() {
        // MAIN DRIVER CONTROLLER BINDS
        m_CommandXboxController.button(Constants.START_BUTTON).onTrue(new PchooOverCS());
        m_CommandXboxController.button(Constants.A_BUTTON).onTrue(new ScoreHybrid());
        m_CommandXboxController.button(Constants.Y_BUTTON).onTrue(new ScoreHighCube());
        m_CommandXboxController.button(Constants.X_BUTTON).onTrue(new ScoreMidCube());
        m_CommandXboxController.button(Constants.B_BUTTON).onTrue(new ScoreMidCone());
        m_CommandXboxController.button(Constants.RIGHT_BUMPER).onTrue(new LoadStation());
        m_CommandXboxController.button(Constants.LEFT_BUMPER).onTrue(new GroundIntake());
        m_CommandXboxController.rightTrigger().onTrue(new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT));
        // m_CommandXboxController.button(Constants.START_BUTTON).toggleOnTrue(new
        // SetLEDColorCommand(0)); Sets LED's to purple
        // m_CommandXboxController.button(Constants.BACK_BUTTON).toggleOnTrue(new
        // SetLEDColorCommand(1)); Sets LED's to yellow

        // // CO-DRIVER CONTROLLER BINDS
        co_CommandXboxController.button(Constants.START_BUTTON).onTrue(new PchooOverCS());
        co_CommandXboxController.button(Constants.A_BUTTON).onTrue(new ScoreHybrid());
        co_CommandXboxController.button(Constants.Y_BUTTON).onTrue(new ScoreHighCube());
        co_CommandXboxController.button(Constants.X_BUTTON).onTrue(new ScoreMidCube());
        co_CommandXboxController.button(Constants.B_BUTTON).onTrue(new ScoreMidCone());
        co_CommandXboxController.button(Constants.RIGHT_BUMPER).onTrue(new LoadStation());
        co_CommandXboxController.button(Constants.LEFT_BUMPER).onTrue(new GroundIntake());
        co_CommandXboxController.axisGreaterThan(1, 0.5).whileTrue(new Retract());
        co_CommandXboxController.axisLessThan(1, -0.5).whileTrue(new Extend());
        co_CommandXboxController.axisGreaterThan(5, 0.5).whileTrue(new MaxTiltDown());
        co_CommandXboxController.axisLessThan(5, -0.5).whileTrue(new MaxTiltUp());
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}
