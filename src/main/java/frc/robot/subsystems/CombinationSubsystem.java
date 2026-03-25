package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class CombinationSubsystem extends SubsystemBase {

    private final ShooterSubsystem m_shooter;
    private final IntakeSubsystem m_intake;
    private final HopperSubsystem m_hopper;

    private boolean gateReversed = false;

    public CombinationSubsystem(ShooterSubsystem shooter, IntakeSubsystem intake, HopperSubsystem hopper) {
        m_shooter = shooter;
        m_intake = intake;
        m_hopper = hopper;
    }
    //--Basic Commands


    public Command cycleSpeedCommand() {
        return m_shooter.cycleShootSpeedCommand();
    }

    public Command armUpCommand() {
        return m_intake.IntakeArmUpCommand(1);
    }

    public Command armDownCommand() {
        return m_intake.IntakeArmDownCommand(1);
    }
    //--Combined Commands

    public Command toggleGateReverseCommand() {
        gateReversed = !gateReversed;
        if (!gateReversed) {
            return m_shooter.gateStopCommand();
        }
        return m_shooter.gateReverseCommand(0.05);
    }


    public Command shrinkCommand() {
        return Commands.sequence (
            m_intake.IntakeArmUpCommand(1),
            m_hopper.hopperDownCommand(2)
        );

    }

    public Command gateReverseAndIntakeCommand() {
        return Commands.sequence(
            m_hopper.hopperUpCommand(2),
            new RunCommand(() -> {
                m_shooter.setGatePower(-0.1);
                m_intake.setIntakeVelocity(0.4);
            }, m_shooter, m_intake)
        );
    }

    public Command gateAndIntakeStopCommand() {
        return new InstantCommand(() -> {
            m_shooter.setGatePower(0);
            m_intake.stop();
        }, m_shooter, m_intake);
    }
    //--Finalized Commands
    

    public Command completeShoot() {
        if (gateReversed) {
            return Commands.sequence(
                m_shooter.gateStopCommand(),
                m_shooter.fullShootCommand()
            );
        } 
        return m_shooter.fullShootCommand();
    }

    public Command completeStop() {
        if (gateReversed) {
            return new InstantCommand(() -> {
                m_shooter.gateStop();
                m_shooter.shooterStop();
                m_shooter.setGatePower(-0.05);
            }, m_shooter);
        }
        return m_shooter.dualStopCommand();
    }


}