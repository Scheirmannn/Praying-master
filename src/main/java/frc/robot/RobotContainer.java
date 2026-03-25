package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController; 
import frc.robot.Constants.SparkConstants;
import frc.robot.autos.driveBackAndShoot;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CombinationSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final HopperSubsystem m_hopper = new HopperSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem(SparkConstants.kLeftShooterCanId, SparkConstants.kRightShooterCanId, SparkConstants.kGateMotorCanId);
    private final IntakeSubsystem m_intake = new IntakeSubsystem(SparkConstants.kLeftIntakeCanId, SparkConstants.kRightIntakeCanId);
    private final CombinationSubsystem m_combo = new CombinationSubsystem(m_shooter, m_intake, m_hopper);

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
            .whileTrue(m_combo.gateReverseAndIntakeCommand())
            .onFalse(m_combo.gateAndIntakeStopCommand());

        new Trigger(() -> m_oppController.getRightTriggerAxis() > 0.1)
            .whileTrue(m_combo.completeShoot())
            .onFalse(m_combo.completeStop());

        new JoystickButton(m_oppController, XboxController.Button.kA.value)
            .onTrue(m_combo.cycleSpeedCommand());

        new JoystickButton(m_oppController, XboxController.Button.kLeftBumper.value)
            .onTrue(m_combo.armUpCommand());

        new JoystickButton(m_oppController, XboxController.Button.kRightBumper.value)
            .onTrue(m_combo.armDownCommand());
    }

    public Command getAutonomousCommand() {
        return new driveBackAndShoot(m_robotDrive, m_shooter, m_intake);
    }
}