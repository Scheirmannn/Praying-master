package frc.robot.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer.AutoWithPose;
import frc.robot.subsystems.CombinationSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class goLadder extends Command implements AutoWithPose {

    private final DriveSubsystem m_drive;
    private final ShooterSubsystem m_shooter;
    private final IntakeSubsystem m_intake;
    private final CombinationSubsystem m_combo;
    private final AutoFactory m_autoFactory;

    private Command m_autoSequence;

    public goLadder(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, CombinationSubsystem combo, AutoFactory autoFactory) {
        m_drive = drive;
        m_shooter = shooter;
        m_intake = intake;
        m_combo = combo;
        m_autoFactory = autoFactory;
        addRequirements(drive, shooter, intake);
    }

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d(3.5, 3.7, new Rotation2d());
    }

    @Override
    public void initialize() {
        m_drive.resetOdometry(getStartingPose());

        m_autoSequence = Commands.sequence(
            m_combo.armDownCommand(),
            m_combo.hopperDownCommand(),
            m_combo.climberUpCommand(),

            m_autoFactory.trajectoryCmd("goLadder"),
            new InstantCommand(() -> m_drive.stopModules(), m_drive),
                
            m_combo.climberDownCommand(),
            m_combo.completeShoot().withTimeout(4),
            m_combo.completeStop()
        );


        m_autoSequence.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        if (m_autoSequence != null)
            m_autoSequence.cancel();
        m_drive.drive(0, 0, 0, false);
        m_shooter.dualStopCommand().schedule();
    }

    @Override
    public boolean isFinished() {
        return m_autoSequence != null && !m_autoSequence.isScheduled();
    }
}