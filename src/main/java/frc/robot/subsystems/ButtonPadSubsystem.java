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
        switch (currentProfile) {
            case PROFILE1: return m_shooter.shooterSpinUpCommand();
            case PROFILE2: return m_shooter.fullShootCommand();
            default: return new InstantCommand();
        }
    }

    public Command button4ReleasedCommand() {
        switch (currentProfile) {
            case PROFILE1: return m_shooter.shooterStopCommand(); 
            case PROFILE2: return m_shooter.dualStopCommand(); 
            default: return new InstantCommand ();
        }
    }

    public Command button5Command() {
        switch (currentProfile) {
            case PROFILE1: return m_shooter.cycleShootSpeedCommand();
            default: return new InstantCommand();
        }
    }

    public Command button6PressedCommand() {
        switch (currentProfile) {
            case PROFILE1: return m_shooter.gateStartCommand();
            case PROFILE2: return m_shooter.gateReverseCommand();
            default: return new InstantCommand();
        }
    }

    public Command button6ReleasedCommand() {
        switch (currentProfile) {
            case PROFILE1: return m_shooter.gateStopCommand();
            case PROFILE2: return m_shooter.gateStopCommand();
            default: return new InstantCommand();
        }
    }

    public Command button7Command() {
        switch (currentProfile) {
            case PROFILE1: return m_shooter.cycleGatePowerCommand();
            default: return new InstantCommand();
        }
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
