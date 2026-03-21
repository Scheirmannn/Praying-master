package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SparkConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ChoreoShootCommand;
import frc.robot.subsystems.ButtonPadSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ClimberSubsystem m_Climber = new ClimberSubsystem();
    private final ShooterSubsystem m_Shooter = new ShooterSubsystem(SparkConstants.kLeftShooterCanId, SparkConstants.kRightShooterCanId, SparkConstants.kGateMotorCanId);
    private final IntakeSubsystem m_Intake = new IntakeSubsystem(SparkConstants.kLeftIntakeCanId, SparkConstants.kRightIntakeCanId);
    private final ButtonPadSubsystem m_ButtonPad = new ButtonPadSubsystem(m_Shooter, m_Intake, m_Climber);
    private final VisionSubsystem m_Vision = new VisionSubsystem();

    private final AutoFactory m_autoFactory = new AutoFactory(
        m_robotDrive::getPose,
        m_robotDrive::resetOdometry,
        m_robotDrive::followTrajectory,
        true,
        m_robotDrive
    );

    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    GenericHID m_oppController = new GenericHID(OIConstants.kDriverControllerPort + 1);

    public RobotContainer() {
        m_Shooter.setVision(m_Vision);

        m_autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
        m_autoChooser.addOption("Choreo Shoot", new ChoreoShootCommand(m_robotDrive, m_Shooter, m_autoFactory, "yourPathName"));
        SmartDashboard.putData("Auto Chooser", m_autoChooser);

        configureButtonBindings();

        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> m_robotDrive.drive(
                    MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(),
                        RobotBase.isSimulation() ? 0.5 : OIConstants.kDriveDeadband),
                    false),
                m_robotDrive));
    }

    private void configureButtonBindings() {
        new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
            .whileTrue(m_robotDrive.alignToTargetCommand(m_Vision));

        new JoystickButton(m_oppController, 1)
            .whileTrue(m_ButtonPad.button1PressedCommand())
            .onFalse(m_ButtonPad.button1ReleasedCommand());

        new JoystickButton(m_oppController, 2)
            .onTrue(m_ButtonPad.button2Command());

        new JoystickButton(m_oppController, 3)
            .onTrue(m_ButtonPad.button3Command());

        new JoystickButton(m_oppController, 4)
            .whileTrue(m_ButtonPad.button4PressedCommand())
            .onFalse(m_ButtonPad.button4ReleasedCommand());

        new JoystickButton(m_oppController, 5)
            .onTrue(m_ButtonPad.button5Command());

        new JoystickButton(m_oppController, 6)
            .whileTrue(m_ButtonPad.button6PressedCommand())
            .onFalse(m_ButtonPad.button6ReleasedCommand());

        new JoystickButton(m_oppController, 7)
            .onTrue(m_ButtonPad.button7Command());

        new JoystickButton(m_oppController, 8)
            .onTrue(m_ButtonPad.button8Command());

        new JoystickButton(m_oppController, 9)
            .onTrue(m_ButtonPad.button9Command());

        new JoystickButton(m_oppController, 10)
            .whileTrue(m_ButtonPad.button10PressedCommand())
            .onFalse(m_ButtonPad.button10ReleasedCommand());
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}