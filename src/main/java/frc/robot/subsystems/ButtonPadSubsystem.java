package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class ButtonPadSubsystem extends SubsystemBase {

    private final ShooterSubsystem m_shooter;
    private final IntakeSubsystem m_intake;
    //private final ClimberSubsystem m_climber;

    private enum profiles {
        PROFILE1,
        PROFILE2
    }

    private profiles currentProfile = profiles.PROFILE1;

    public ButtonPadSubsystem(ShooterSubsystem shooter, IntakeSubsystem intake, ClimberSubsystem climber) {
        m_shooter = shooter;
        m_intake = intake;
        //m_climber = climber;
        SmartDashboard.putString("Current Profile", currentProfile.name());
    }

    public void toggleProfiles() {
        profiles[] profile = profiles.values();
        int next = (currentProfile.ordinal() + 1) % profile.length;
        currentProfile = profile[next];
        SmartDashboard.putString("Current Profile", currentProfile.name());
    }

    // ── Button commands ──────────────────────────────────────────

    public Command button1PressedCommand() {
        return new RunCommand(() -> {
            m_shooter.setGatePower(-0.1);
            m_intake.setIntakeVelocity(0.33);
        }, m_shooter, m_intake);

    }

    public Command button1ReleasedCommand() {
        return new InstantCommand(() -> {
                m_shooter.setGatePower(0);
                m_intake.stop();
        }, m_shooter, m_intake);
    }

    public Command button2Command() {
        return new InstantCommand();
    }

    public Command button3Command() {
        return new InstantCommand();
    }

    private Command m_currentShooterCommand = null;

    public Command button4PressedCommand() {
        return new InstantCommand(() -> {
            switch (currentProfile) {
                case PROFILE1: 
                    m_currentShooterCommand = m_shooter.shooterSpinUpCommand();
                    CommandScheduler.getInstance().schedule(m_currentShooterCommand); 
                    break;
                default: break;
            }
        }, this);
    }

    public Command button4ReleasedCommand() {
        return new InstantCommand(() -> {
            if (m_currentShooterCommand != null) {
                CommandScheduler.getInstance().cancel(m_currentShooterCommand);
                m_currentShooterCommand = null;
            }
            switch (currentProfile) {
                case PROFILE1: CommandScheduler.getInstance().schedule(m_shooter.shooterStopCommand()); break;
                default: break;
            }
        }, this);
    }

    public Command button5Command() {
        return new InstantCommand(() -> {
            switch (currentProfile) {
                case PROFILE1: CommandScheduler.getInstance().schedule(m_shooter.cycleShootSpeedCommand()); break;
                default: break;
            }
        }, this);
    }

    public Command button6PressedCommand() {
        return new InstantCommand(() -> {
            switch (currentProfile) {
                case PROFILE1: CommandScheduler.getInstance().schedule(m_shooter.gateStartCommand()); break;
                default: break;
            }
        }, this);
    }

    public Command button6ReleasedCommand() {
        return new InstantCommand(() ->
            CommandScheduler.getInstance().schedule(m_shooter.gateStopCommand()), this);
    }

    public Command button7Command() {
        return new InstantCommand(() -> {
            switch (currentProfile) {
                case PROFILE1: CommandScheduler.getInstance().schedule(m_shooter.cycleGatePowerCommand()); break;
                default: break;
            }
        }, this);
    }

    public Command button8Command() {
        return new InstantCommand(() -> {
            switch (currentProfile) {
                case PROFILE1: CommandScheduler.getInstance().schedule(m_intake.IntakeArmUpCommand(1)); break;
                default: break;
            }
        }, this);
    }

    public Command button9Command() {
        return new InstantCommand(() -> {
            switch (currentProfile) {
                case PROFILE1: CommandScheduler.getInstance().schedule(m_intake.IntakeArmDownCommand(1)); break;
                default: break;
            }
        }, this);
    }

    public Command button10PressedCommand() {
        return new InstantCommand(() -> {
            switch (currentProfile) {
                case PROFILE1: CommandScheduler.getInstance().schedule(m_intake.setIntakeCommand(0.5)); break;
                default: break;
            }
        }, this);
    }

    public Command button10ReleasedCommand() {
        return new InstantCommand(() -> {
            switch (currentProfile) {
                case PROFILE1: CommandScheduler.getInstance().schedule(m_intake.stopIntakeCommand()); break;
                default: break;
            }
        }, this);
    }
}