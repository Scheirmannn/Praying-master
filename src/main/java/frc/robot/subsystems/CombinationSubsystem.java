package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;

public class CombinationSubsystem extends SubsystemBase {

    private final ShooterSubsystem m_shooter;
    private final IntakeSubsystem m_intake;
    private final HopperSubsystem m_hopper;
    private final ClimberSubsystem m_climber;

    private boolean gateReversed = false;

    public CombinationSubsystem(ShooterSubsystem shooter, IntakeSubsystem intake, HopperSubsystem hopper, ClimberSubsystem climber) {
        m_shooter = shooter;
        m_intake = intake;
        m_hopper = hopper;
        m_climber = climber;
    }
    //--Basic Commands

    public Command hopperUpCommand() {
        return m_hopper.hopperUpCommand(2);
    }

    public Command hopperDownCommand() {
        return Commands.defer(() -> {
            if (m_intake.m_intakeUp) {
                return new InstantCommand();
            }
            return m_hopper.hopperDownCommand(1);
        }, Set.of(m_hopper));
    }

    public Command climberUpCommand() {
        return m_climber.climbUpCommand(2);
    }

    public Command climberDownCommand() {
        return m_climber.climbDownCommand(2);
    }

    public Command armUpCommand() {
        return m_intake.IntakeArmUpCommand(1);
    }

    public Command armDownCommand() {
        return m_intake.IntakeArmDownCommand(1);
    }

    public Command cycleSpeedCommand() {
        return m_shooter.cycleShootSpeedCommand();
    }

    public Command visionToggle() {
        return m_shooter.visionToggle();
    }

    //--Combined Commands

    public Command startingCommand() {
        return Commands.sequence(
            armDownCommand(),
            hopperDownCommand()
        );
    }

    public Command setGateReversedCommand(boolean reversed) {
        return new InstantCommand(() -> {
            gateReversed = reversed;
            if (reversed) {
                m_shooter.setGatePower(-0.05);
            } else {
                m_shooter.gateStop();
            }
        }, m_shooter);
    }

    public Command gateReverseAndIntakeCommand() {
        return Commands.defer(() -> {
            if (!m_hopper.m_out) {
                return Commands.sequence(
                    m_hopper.hopperUpCommand(2),
                    new RunCommand(() -> {
                        m_shooter.setGatePower(-0.1);
                        m_intake.setIntakeVelocity(0.4);
                    }, m_shooter, m_intake)
                );
            }
            return new RunCommand(() -> {
                m_shooter.setGatePower(-0.1);
                m_intake.setIntakeVelocity(0.4);
            }, m_shooter, m_intake);

        }, Set.of(m_shooter, m_intake, m_hopper));
    }

    public Command gateAndIntakeStopCommand() {
        return new InstantCommand(() -> {
            m_shooter.setGatePower(0);
            m_intake.stop();
        }, m_shooter, m_intake);
    }
    
    //--Finalized Commands

    public Command completeIntake() {
        return Commands.defer(() -> {
            if (gateReversed) {
                return Commands.sequence(
                    m_shooter.gateStopCommand(),
                    gateReverseAndIntakeCommand()
                );
            }
            return gateReverseAndIntakeCommand();
        }, Set.of(m_shooter, m_intake, m_hopper));
    }

    public Command completeIntakeStop() {
        return Commands.defer(() -> {
            if (gateReversed) {
                return Commands.sequence(
                    gateAndIntakeStopCommand(),
                    m_shooter.gateReverseCommand(0.05)
                );
            }
            return gateAndIntakeStopCommand();
        }, Set.of(m_shooter, m_intake));
    }

    public Command completeShoot() {
        return Commands.defer(() -> {
            if (gateReversed) {
                return Commands.sequence(
                    m_shooter.gateStopCommand(),
                    m_shooter.fullShootCommand()
                );
            }
            return m_shooter.fullShootCommand();
        }, Set.of(m_shooter));
    }

    public Command completeShooterStop() {
        return Commands.defer(() -> {
            if (gateReversed) {
                return new InstantCommand(() -> {
                    m_shooter.gateStop();
                    m_shooter.setShooterVelocity(0);
                    m_shooter.setGatePower(-0.05);
                }, m_shooter);
            }
            return m_shooter.dualStopCommand();
        }, Set.of(m_shooter));
    }


}