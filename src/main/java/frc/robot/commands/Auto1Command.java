package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Auto1Command extends Command {

    private final DriveSubsystem m_drive;
    private final ShooterSubsystem m_shooter;
    private final IntakeSubsystem m_intake;

    private Command m_autoSequence;

    public Auto1Command(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake) {
        m_drive = drive;
        m_shooter = shooter;
        m_intake = intake;
        addRequirements(drive, shooter, intake);
    }

    @Override
    public void initialize() {
        m_autoSequence = Commands.sequence(

            new InstantCommand(() -> m_shooter.setSpeedProfile(ShooterSubsystem.speedProfiles.AUTO)),

            // Drive backward to first shooting position
            new RunCommand(() -> m_drive.drive(-0.3, 0, 0, false), m_drive)
                .withTimeout(2.0),

            new InstantCommand(() -> m_drive.drive(0, 0, 0, false), m_drive),

            // Shoot 8 balls
            Commands.parallel(
                new RunCommand(() -> m_shooter.setShooterVelocity(m_shooter.getTargetSpeed()), m_shooter),
                Commands.sequence(
                    Commands.waitUntil(m_shooter::isAtSpeed),
                    Commands.waitSeconds(0.5),
                    new RunCommand(() -> m_shooter.setGatePower(m_shooter.currentGatePower()), m_shooter)
                        .withTimeout(4.0)
                )
            ).withTimeout(7.0),

            m_shooter.dualStopCommand(),

            // Drive diagonally while deploying intake
            Commands.parallel(
                new RunCommand(() -> m_drive.drive(-0.24, 0.18, 0, false), m_drive)
                    .withTimeout(2.5),
                m_intake.IntakeArmDownCommand(2.5)
            ),

            // Collect balls
            new RunCommand(() -> {
                m_drive.drive(0, 0, 0, false);
                m_intake.setIntakeVelocity(0.5);
            }, m_drive, m_intake).withTimeout(3.0),

            // Stop intake and raise arm
            new InstantCommand(() -> {
                m_intake.stop();
                m_intake.setArmUp();
            }, m_intake),

            // Shoot from current position
            Commands.parallel(
                new RunCommand(() -> m_shooter.setShooterVelocity(m_shooter.getTargetSpeed()), m_shooter),
                Commands.sequence(
                    Commands.waitUntil(m_shooter::isAtSpeed),
                    Commands.waitSeconds(0.5),
                    new RunCommand(() -> m_shooter.setGatePower(m_shooter.currentGatePower()), m_shooter)
                        .withTimeout(4.0)
                )
            ).withTimeout(7.0),

            m_shooter.dualStopCommand()
        );

        m_autoSequence.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        if (m_autoSequence != null) {
            m_autoSequence.cancel();
        }
        m_drive.drive(0, 0, 0, false);
        m_shooter.dualStopCommand().schedule();
    }

    @Override
    public boolean isFinished() {
        return m_autoSequence != null && !m_autoSequence.isScheduled();
    }
}