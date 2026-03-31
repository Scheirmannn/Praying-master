package frc.robot.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class goBack extends Command{

    private final DriveSubsystem m_drive;
    private final ShooterSubsystem m_shooter;
    private final IntakeSubsystem m_intake;
    private final AutoFactory m_autoFactory;

    private Command m_autoSequence;

    public goBack(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, AutoFactory autoFactory) {
        m_drive = drive;
        m_shooter = shooter;
        m_intake = intake;
        m_autoFactory = autoFactory;
        addRequirements(drive, shooter, intake);
    }

    @Override
    public void initialize() {

        m_autoSequence = Commands.sequence(
            new InstantCommand(() -> m_intake.setArmDown()),

            m_autoFactory.trajectoryCmd("goBack"),
            new InstantCommand(() -> m_drive.stopModules(), m_drive),

            m_shooter.fullShootCommand().withTimeout(8.0),
            m_shooter.dualStopCommand()
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