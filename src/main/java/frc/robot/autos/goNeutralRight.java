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
import frc.robot.subsystems.ShooterSubsystem;

public class goNeutralRight extends Command implements AutoWithPose {

    private final DriveSubsystem m_drive;
    private final CombinationSubsystem m_combo;
    private final ShooterSubsystem m_shooter;
    private final AutoFactory m_autoFactory;

    private Command m_autoSequence;

    public goNeutralRight(DriveSubsystem drive, CombinationSubsystem combo, ShooterSubsystem shooter, AutoFactory autoFactory) {
        m_drive = drive;
        m_combo = combo;
        m_shooter = shooter;
        m_autoFactory = autoFactory;
        addRequirements(drive);  // combo subsystems handle their own requirements
    }

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d(3.5, 5.8, new Rotation2d());
    }

    @Override
    public void initialize() {
        m_drive.resetOdometry(getStartingPose());

        m_autoSequence = Commands.sequence(
            m_combo.startingCommand(),
            m_combo.setGateReversedCommand(true),
                
            m_autoFactory.trajectoryCmd("goNeutralRightPt1"),
            new InstantCommand(() -> m_drive.stopModules(), m_drive),

            m_combo.completeShoot().withTimeout(4),
            m_combo.completeShooterStop(),

            m_autoFactory.trajectoryCmd("goNeutralRightPt2"),
            new InstantCommand(() -> m_drive.stopModules(), m_drive),
        
            m_combo.hopperUpCommand(),

            Commands.race(
                m_combo.gateReverseAndIntakeCommand(),
                m_autoFactory.trajectoryCmd("goNeutralRightPt3")         
            ),

            new InstantCommand(() -> m_drive.stopModules(), m_drive),
            m_combo.gateAndIntakeStopCommand(),

            m_shooter.fullShootCommand().withTimeout(6.7));
        m_autoSequence.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        if (m_autoSequence != null)
            m_autoSequence. cancel();
        m_drive.drive(0, 0, 0, false);
        m_shooter.dualStopCommand().schedule();
    }

    @Override
    public boolean isFinished() {
        return m_autoSequence != null && !m_autoSequence.isScheduled();
    }
}