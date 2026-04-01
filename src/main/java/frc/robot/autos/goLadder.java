package frc.robot.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CombinationSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class goLadder extends Command {

    private final DriveSubsystem m_drive;
    private final CombinationSubsystem m_combo;
    private final AutoFactory m_autoFactory;

    private Command m_autoSequence;

    public goLadder(DriveSubsystem drive, CombinationSubsystem combo, AutoFactory autoFactory) {
        m_drive = drive;
        m_combo = combo;
        m_autoFactory = autoFactory;
        addRequirements(drive);
    }


    @Override
    public void initialize() {

        m_autoSequence = Commands.sequence(
            m_combo.armDownCommand(),
            m_combo.hopperDownCommand(),
            m_combo.climberUpCommand(),

            m_autoFactory.trajectoryCmd("goLadder"),
            new InstantCommand(() -> m_drive.stopModules(), m_drive),
                
            m_combo.climberDownCommand(),
            m_combo.completeShoot().withTimeout(4),
            m_combo.completeShooterStop()
        );


        CommandScheduler.getInstance().schedule(m_autoSequence);
    }

    @Override
    public void end(boolean interrupted) {
        if (m_autoSequence != null)
            m_autoSequence.cancel();
        m_drive.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return m_autoSequence != null && !m_autoSequence.isScheduled();
    }
}