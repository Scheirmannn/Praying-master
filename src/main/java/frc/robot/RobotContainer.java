package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SparkConstants;
import frc.robot.autos.goBack;
import frc.robot.autos.goDepot;
import frc.robot.autos.goLadder;
import frc.robot.autos.goNeutralRight;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CombinationSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final HopperSubsystem m_hopper = new HopperSubsystem();
    private final ClimberSubsystem m_climber = new ClimberSubsystem();
    private final Vision m_vision = new Vision(m_robotDrive::addVisionMeasurement);
    private final ShooterSubsystem m_shooter = new ShooterSubsystem(SparkConstants.kLeftShooterCanId, SparkConstants.kRightShooterCanId, SparkConstants.kGateMotorCanId);
    private final IntakeSubsystem m_intake = new IntakeSubsystem(SparkConstants.kLeftIntakeCanId, SparkConstants.kRightIntakeCanId);
    private final CombinationSubsystem m_combo = new CombinationSubsystem(m_shooter, m_intake, m_hopper, m_climber);

    private AutoFactory m_autoFactory;
    
    
    public interface AutoWithPose {
        Pose2d getStartingPose();
    }

    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    XboxController m_oppController = new XboxController(OIConstants.kDriverControllerPort + 1);

    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    public boolean fieldRelative = true;

    public RobotContainer() {
        m_shooter.useVision(m_vision);

        m_autoFactory = new AutoFactory(
                m_robotDrive::getPose,
                m_robotDrive::resetOdometry,
                m_robotDrive::followTrajectory,
                true,
                m_robotDrive);

        m_autoChooser.setDefaultOption("goBack", new goBack(m_robotDrive, m_shooter, m_intake, m_autoFactory));   
        m_autoChooser.addOption("goLadder", new goLadder(m_robotDrive, m_shooter, m_combo, m_autoFactory));
        m_autoChooser.addOption("goDepot", new goDepot(m_robotDrive, m_shooter, m_intake, m_autoFactory));
        m_autoChooser.addOption("goNeutralRight", new goNeutralRight(m_robotDrive, m_combo, m_shooter, m_autoFactory));
        m_autoChooser.addOption("Do Nothing", new InstantCommand());
        SmartDashboard.putData("Auto Chooser", m_autoChooser);

        configureButtonBindings();

        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            // NOTE: Using robot-relative drive (fieldRelative = false) for normal teleop
            new RunCommand(
                () -> {
                    double xSpeed = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);
                    double ySpeed = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband);
                    double rot = -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband);

                    // Robot-relative drive: false = robot-relative, true = field-relative
                    SmartDashboard.putString("Drive Mode", fieldRelative ? "Field-Relative" : "Robot-Relative");
                    m_robotDrive.drive(xSpeed, ySpeed, rot, fieldRelative);

                }, m_robotDrive)
        );
        }

    private void configureButtonBindings() {

        new JoystickButton(m_driverController, XboxController.Button.kBack.value)
            .onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative));

        new JoystickButton(m_driverController, XboxController.Button.kY.value)
            .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

        new JoystickButton(m_driverController, XboxController.Button.kStart.value)
            .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(), m_robotDrive));


        
        new Trigger(() -> m_oppController.getLeftTriggerAxis() > 0.1)
            .whileTrue(m_combo.completeIntake())
            .onFalse(m_combo.completeIntakeStop());

        new Trigger(() -> m_oppController.getRightTriggerAxis() > 0.1)
            .whileTrue(m_combo.completeShoot())
            .onFalse(m_combo.completeShooterStop());

        new JoystickButton(m_oppController, XboxController.Button.kA.value)
            .onTrue(m_combo.cycleSpeedCommand());

        new JoystickButton(m_oppController, XboxController.Button.kY.value)
            .onTrue(m_combo.visionToggle());

        new JoystickButton(m_oppController, XboxController.Button.kLeftBumper.value)
            .onTrue(m_combo.armUpCommand());

        new JoystickButton(m_oppController, XboxController.Button.kRightBumper.value)
            .onTrue(m_combo.armDownCommand());
        
        new Trigger(() -> m_oppController.getPOV() == 0)
            .onTrue(m_combo.climberUpCommand());

        new Trigger(() -> m_oppController.getPOV() == 180)
            .onTrue(m_combo.climberDownCommand());

        new Trigger(() -> m_oppController.getPOV() == 90)
            .onTrue(m_combo.hopperUpCommand());

        new Trigger(() -> m_oppController.getPOV() == 270)
            .onTrue(m_combo.hopperDownCommand());   
        }

    public DriveSubsystem getDrivetrain() {
        return m_robotDrive;
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

    public void applySimStartingPose() {
        if (!RobotBase.isSimulation())
            return;
        Command selected = m_autoChooser.getSelected();
        if (selected instanceof AutoWithPose auto) {
            m_robotDrive.setSimStartingPose(auto.getStartingPose());
        }
    }
}