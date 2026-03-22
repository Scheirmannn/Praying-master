package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController; 
import frc.robot.Constants.SparkConstants;
import frc.robot.commands.driveBackAndShoot;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ButtonPadSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ClimberSubsystem m_Climber = new ClimberSubsystem();
    private final ShooterSubsystem m_Shooter = new ShooterSubsystem(SparkConstants.kLeftShooterCanId, SparkConstants.kRightShooterCanId, SparkConstants.kGateMotorCanId);
    private final IntakeSubsystem m_Intake = new IntakeSubsystem(SparkConstants.kLeftIntakeCanId, SparkConstants.kRightIntakeCanId);
    private final ButtonPadSubsystem m_ButtonPad = new ButtonPadSubsystem(m_Shooter, m_Intake, m_Climber);

    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    XboxController m_oppController = new XboxController(OIConstants.kDriverControllerPort + 1);

    public RobotContainer() {

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

        new Trigger(() -> m_oppController.getLeftTriggerAxis() > 0.1)
            .whileTrue(m_ButtonPad.button1PressedCommand())
            .onFalse(m_ButtonPad.button1ReleasedCommand());

        new Trigger(() -> m_oppController.getRightTriggerAxis() > 0.1)
            .whileTrue(m_Shooter.fullShootCommand())
            .onFalse(m_Shooter.dualStopCommand());

        new JoystickButton(m_oppController, XboxController.Button.kA.value)
            .onTrue(m_ButtonPad.button5Command());

        new JoystickButton(m_oppController, XboxController.Button.kLeftBumper.value)
            .onTrue(m_ButtonPad.button8Command());

        new JoystickButton(m_oppController, XboxController.Button.kRightBumper.value)
            .onTrue(m_ButtonPad.button9Command());
    }

    public Command getAutonomousCommand() {
        return new driveBackAndShoot(m_robotDrive, m_Shooter);
    }
}