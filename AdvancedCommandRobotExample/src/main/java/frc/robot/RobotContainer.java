package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.TiltSubsystem;
import frc.robot.commands.PIDCommand.TiltPIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SimpleCommands.Extend;
import frc.robot.commands.SimpleCommands.Intake;
import frc.robot.commands.SimpleCommands.MaxTiltUp;
import frc.robot.commands.SimpleCommands.Retract;
import frc.robot.commands.SimpleCommands.MaxTiltDown;
import frc.robot.commands.Autonomous.TimedBasedAuto.Auto1;

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

        private final Command TwoCubeChargeStation = new Auto1();
        private final Command TestAuto = new Auto2();
        private final Command TestAuto = new Auto3();

        SendableChooser<Command> m_chooser = new SendableChooser<>();

        public RobotContainer() {
                configureBindings();
                driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());

                m_chooser.addOption("Test Auto", TestAuto);

                SmartDashboard.putData(m_chooser);
        }

        private void configureBindings() {
                // MAIN DRIVER CONTROLLER BINDS
                m_CommandXboxController.button(Constants.BACK_BUTTON)
                                .whileTrue(new Intake(Constants.MANIPULATOR_SPEED_PCHOO));
                m_CommandXboxController.button(Constants.A_BUTTON)
                                .onTrue(new TiltPIDCommand(Constants.TILT_HYBRID_SCORE_SETPOINT).withTimeout(0.5)
                                                .andThen(new Intake(Constants.MANIPULATOR_SPEED_OUTTAKE)
                                                                .withTimeout(0.5))
                                                .andThen(new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT)
                                                                .withTimeout(0.7)));
                m_CommandXboxController.button(Constants.Y_BUTTON) // Score High Cube
                                .onTrue(new TiltPIDCommand(Constants.TILT_LOAD_STATION_SETPOINT).withTimeout(0.5)
                                                .andThen(new Extend().withTimeout(0.5))
                                                .andThen(new Intake(Constants.MANIPULATOR_SPEED_BLEH).withTimeout(0.5))
                                                .andThen(new Retract().withTimeout(0.5))
                                                .andThen(new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT)
                                                                .withTimeout(0.5)));

                m_CommandXboxController.button(Constants.X_BUTTON) // Score Mid Cube
                                .onTrue(new TiltPIDCommand(Constants.TILT_MID_SCORE_SETPOINT).withTimeout(0.5)
                                                .andThen(new Intake(Constants.MANIPULATOR_SPEED_BLEH).withTimeout(0.5))
                                                .andThen(new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT)
                                                                .withTimeout(0.5)));

                m_CommandXboxController.rightTrigger().onTrue(new TiltPIDCommand(Constants.TILT_UP_SETPOINT));
                // Score Cone
                m_CommandXboxController.button(Constants.B_BUTTON)
                                .onTrue(new TiltPIDCommand(Constants.TILT_LOAD_STATION_SETPOINT).withTimeout(0.1)
                                                .andThen(new Extend().withTimeout(0.6))
                                                .andThen(new TiltPIDCommand(Constants.TILT_MID_SCORE_SETPOINT)
                                                                .withTimeout(0.5))
                                                .andThen(new Retract().withTimeout(0.5))
                                                .andThen(new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT)
                                                                .withTimeout(0.5)));

                m_CommandXboxController.button(Constants.RIGHT_BUMPER).onTrue(Commands // Load from load station
                                .parallel(new TiltPIDCommand(Constants.TILT_LOAD_STATION_SETPOINT),
                                                new Intake(Constants.MANIPULATOR_SPEED_INTAKE)));
                m_CommandXboxController.button(Constants.LEFT_BUMPER).onTrue(Commands
                                .parallel(new TiltPIDCommand(Constants.TILT_LOW_SETPOINT),
                                                new Intake(Constants.MANIPULATOR_SPEED_INTAKE)));

                // m_CommandXboxController.button(Constants.START_BUTTON).toggleOnTrue(new
                // SetLEDColorCommand(0)); Sets LED's to purple
                // m_CommandXboxController.button(Constants.BACK_BUTTON).toggleOnTrue(new
                // SetLEDColorCommand(1)); Sets LED's to yellow

                // CO-DRIVER CONTROLLER BINDS
                co_CommandXboxController.button(Constants.A_BUTTON)
                                .onTrue(new TiltPIDCommand(Constants.TILT_HYBRID_SCORE_SETPOINT).withTimeout(0.5)
                                                .andThen(new Intake(Constants.MANIPULATOR_SPEED_OUTTAKE)
                                                                .withTimeout(0.5))
                                                .andThen(new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT)
                                                                .withTimeout(0.7)));
                co_CommandXboxController.button(Constants.Y_BUTTON) // Score High Cube
                                .onTrue(new TiltPIDCommand(Constants.TILT_LOAD_STATION_SETPOINT).withTimeout(0.5)
                                                .andThen(new Extend().withTimeout(0.5))
                                                .andThen(new Intake(Constants.MANIPULATOR_SPEED_BLEH).withTimeout(0.5))
                                                .andThen(new Retract().withTimeout(0.5))
                                                .andThen(new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT)
                                                                .withTimeout(0.5)));

                co_CommandXboxController.button(Constants.X_BUTTON) // Score Mid Cube
                                .onTrue(new TiltPIDCommand(Constants.TILT_MID_SCORE_SETPOINT).withTimeout(0.5)
                                                .andThen(new Intake(Constants.MANIPULATOR_SPEED_BLEH).withTimeout(0.5))
                                                .andThen(new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT)
                                                                .withTimeout(0.5)));

                co_CommandXboxController.rightTrigger().onTrue(new TiltPIDCommand(Constants.TILT_UP_SETPOINT));
                // Score Cone
                co_CommandXboxController.button(Constants.B_BUTTON)
                                .onTrue(new TiltPIDCommand(Constants.TILT_LOAD_STATION_SETPOINT).withTimeout(0.1)
                                                .andThen(new Extend().withTimeout(0.6))
                                                .andThen(new TiltPIDCommand(Constants.TILT_MID_SCORE_SETPOINT)
                                                                .withTimeout(0.5))
                                                .andThen(new Retract().withTimeout(0.5))
                                                .andThen(new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT)
                                                                .withTimeout(0.5)));

                co_CommandXboxController.button(Constants.RIGHT_BUMPER).onTrue(Commands // Load from load station
                                .parallel(new TiltPIDCommand(Constants.TILT_LOAD_STATION_SETPOINT),
                                                new Intake(Constants.MANIPULATOR_SPEED_INTAKE)));
                co_CommandXboxController.button(Constants.LEFT_BUMPER).onTrue(Commands
                                .parallel(new TiltPIDCommand(Constants.TILT_LOW_SETPOINT),
                                                new Intake(Constants.MANIPULATOR_SPEED_INTAKE)));
                co_CommandXboxController.axisGreaterThan(1, 0.5).whileTrue(new Retract());
                co_CommandXboxController.axisLessThan(1, -0.5).whileTrue(new Extend());
                co_CommandXboxController.axisGreaterThan(5, 0.5).whileTrue(new MaxTiltDown());
                co_CommandXboxController.axisLessThan(5, -0.5).whileTrue(new MaxTiltUp());
        }

        public Command getAutonomousCommand() {
                return m_chooser.getSelected();
        }
}
