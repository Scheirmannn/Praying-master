package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;


public class ButtonPadSubsystem extends SubsystemBase{

    private final ShooterSubsystem m_shooter;
    private final IntakeSubsystem m_intake;
    private final ClimberSubsystem m_climber;

    private enum profiles {
        PROFILE1,
        PROFILE2

    }

    private profiles currentProfile = profiles.PROFILE1;
    
    public ButtonPadSubsystem ( ShooterSubsystem  shooter, IntakeSubsystem intake, ClimberSubsystem climber) {
        
        m_shooter = shooter;
        m_intake = intake;
        m_climber = climber;


        SmartDashboard.putString("Current profile", currentProfile.name());
    }

    public void toggleProfiles() {
        profiles[] profile = profiles.values();
        int next = (currentProfile.ordinal() + 1) % profile.length;
        currentProfile = profile[next];
        SmartDashboard.putString("Current profile", currentProfile.name());
    }

    public Command button1Command() {
        return new InstantCommand(() -> toggleProfiles(), this);
    }

    public Command button2Command() {
        return m_climber.climbUpCommand(2);
    }

    public Command button3Command() {
        return m_climber.climbDownCommand(2);
    }

    public Command button4PressedCommand() {
        return new RunCommand(() -> {
            switch (currentProfile) {
                case PROFILE1: m_shooter.shooterSpinUpCommand(); break;
                case PROFILE2: m_shooter.fullShootCommand(); break;
                default: break;
            }
        }, this);
    }

    public Command button4ReleasedCommand() {
        return new InstantCommand(() -> {
            switch (currentProfile) {
                case PROFILE1: m_shooter.shooterStop(); break;
                case PROFILE2: m_shooter.dualStopCommand(); break;
                default: break;
            }
        }, this);
    }

    public Command button5Command() {
        return new InstantCommand(() -> {
            switch (currentProfile) {
                case PROFILE1: m_shooter.cycleShootSpeedCommand(); break;
                case PROFILE2: break;
                default: break;
            }
        }, this);
    }

    public Command button6PressedCommand() {
        return new InstantCommand(() -> {
            switch (currentProfile) {
                case PROFILE1: m_shooter.gateStartCommand(); break;
                case PROFILE2: m_shooter.gateReverseCommand(); break;
                default: break;
            }
        }, this);
    }

    public Command button6ReleasedCommand() {
        return new InstantCommand(() -> {
            switch (currentProfile) {
                case PROFILE1: m_shooter.gateStopCommand(); break;
                case PROFILE2: m_shooter.gateStopCommand(); break;
                default: break;
            }
        }, this);
    }

    public Command button7Command() {
        return new InstantCommand(() -> {
            switch (currentProfile) {
                case PROFILE1: m_shooter.cycleGatePowerCommand(); break;
                case PROFILE2: break;
                default: break;
            }
        }, this);
    }
    
    public Command button8Command() {
        return m_intake.IntakeArmUpCommand(1);
    }

    public Command button9Command() {
        return m_intake.IntakeArmDownCommand(2.5);
    }

    public Command button10PressedCommand() {
        return m_intake.setIntakeCommand(.5);
    }

    public Command button10ReleasedCommand() {
        return m_intake.intakeArmStopCommand();
    }



}
