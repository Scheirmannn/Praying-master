//everything for the shooter prob
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DoubleSolenoid;


public class HopperSubsystem extends SubsystemBase {

    private final DoubleSolenoid climberSolenoid;

    public boolean m_out = true;

    public HopperSubsystem() {
        climberSolenoid = new DoubleSolenoid(13, PneumaticsModuleType.CTREPCM, 2, 3);

    }

    public boolean HopperUp() {
        return m_out;
    }

    public void setHopperOut() {
        climberSolenoid.set(DoubleSolenoid.Value.kForward);
        m_out = true;
    }

    public void setHopperIn() {
        climberSolenoid.set(DoubleSolenoid.Value.kReverse);
        m_out = false;
    }

     public void setHopperStop() {
        climberSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    public Command hopperStopCommand() {
        return new InstantCommand(() -> setHopperStop(), this);
    }

    public Command hopperDownCommand(double time) {
        return Commands.sequence(
            new InstantCommand(() -> { setHopperIn(); }, this),
            Commands.waitSeconds(time),
            hopperStopCommand()
        );
    }

    public Command hopperUpCommand(double time) {
      return Commands.sequence(
              new InstantCommand(() -> { setHopperIn(); }, this),
            Commands.waitSeconds(time),
            hopperStopCommand()
        ); 
    }

}