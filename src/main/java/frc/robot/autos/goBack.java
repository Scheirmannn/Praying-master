package frc.robot.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CombinationSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class goBack extends Command{

    private final DriveSubsystem m_drive;
    private final CombinationSubsystem m_combo;
    private final AutoFactory m_autoFactory;

    private Command m_autoSequence;

    public goBack(DriveSubsystem drive, CombinationSubsystem combo,  AutoFactory autoFactory) {
        m_drive = drive;
        m_combo = combo;
        m_autoFactory = autoFactory;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_autoSequence = Commands.sequence(
            m_combo.setGateReversedCommand(true),

            m_combo.startingCommand(), 
            m_autoFactory.trajectoryCmd("goBack"),

            new InstantCommand(() -> m_drive.stopModules(), m_drive),

            m_combo.completeShoot().withTimeout(15)
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