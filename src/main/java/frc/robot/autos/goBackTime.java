package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.CombinationSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class goBackTime extends Command {

    private final DriveSubsystem m_drive;
    private final CombinationSubsystem m_combo;

    private Command m_autoSequence;

    public goBackTime(DriveSubsystem drive, CombinationSubsystem combo) {
        m_drive = drive;
        m_combo = combo;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_autoSequence = Commands.sequence(    
            new RunCommand(() -> m_drive.drive(-0.3, 0, 0, false), m_drive).withTimeout(1.0),

            new InstantCommand(() -> m_drive.drive(0, 0, 0, false), m_drive),

            m_combo.completeShoot().withTimeout(8.0),

            m_combo.completeShooterStop()
        );

        CommandScheduler.getInstance().schedule(m_autoSequence);
    }

    @Override
    public void end(boolean interrupted) {
        if (m_autoSequence != null)
            m_autoSequence.cancel();
        m_drive.drive(0, 0, 0, false);
        CommandScheduler.getInstance().schedule(m_combo.completeShooterStop());
    }

    @Override
    public boolean isFinished() {
        return m_autoSequence != null && !m_autoSequence.isScheduled();
    }
}