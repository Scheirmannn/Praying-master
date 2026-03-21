package frc.robot.commands;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ChoreoShootCommand extends Command {

    private final DriveSubsystem m_drive;
    private final ShooterSubsystem m_shooter;
    private final AutoFactory m_autoFactory;
    private final String m_pathName;

    private Command m_autoSequence;

    public ChoreoShootCommand(DriveSubsystem drive, ShooterSubsystem shooter, AutoFactory autoFactory, String pathName) {
        m_drive = drive;
        m_shooter = shooter;
        m_autoFactory = autoFactory;
        m_pathName = pathName;
        addRequirements(drive, shooter);
    }

    @Override
    public void initialize() {
        m_autoSequence = Commands.sequence(
            m_autoFactory.trajectoryCmd(m_pathName),
            new InstantCommand(() -> m_drive.drive(0, 0, 0, false), m_drive),
            m_shooter.fullShootCommand()
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (m_autoSequence != null) m_autoSequence.cancel();
        m_drive.drive(0, 0, 0, false);
        m_shooter.dualStopCommand().schedule();
    }

    @Override
    public boolean isFinished() {
        return m_autoSequence != null && !m_autoSequence.isScheduled();
    }
}