package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class driveBackAndShoot extends Command {

    private final DriveSubsystem m_drive;
    private final ShooterSubsystem m_shooter;
    private final IntakeSubsystem m_intake;

    private Command m_autoSequence;

    public driveBackAndShoot(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake) {
        m_drive = drive;
        m_shooter = shooter;
        m_intake = intake;
        addRequirements(drive, shooter, intake);
    }

    @Override
    public void initialize() {
        m_autoSequence = Commands.sequence(
            new InstantCommand(() -> m_intake.setArmDown()),

            
        
            new RunCommand(() -> m_drive.drive(-0.3, 0, 0, false), m_drive)
                .withTimeout(2.0),

            new InstantCommand(() -> m_drive.drive(0, 0, 0, false), m_drive),

            m_shooter.fullShootCommand().withTimeout(8.0),

            m_shooter.dualStopCommand()
        );
        m_autoSequence.schedule();
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