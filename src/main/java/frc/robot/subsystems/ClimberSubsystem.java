//everything for the shooter prob
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DoubleSolenoid;


public class ClimberSubsystem extends SubsystemBase {

    private final DoubleSolenoid climberSolenoid;

    public boolean m_climbed = false;

    public ClimberSubsystem() {
        climberSolenoid = new DoubleSolenoid(13, PneumaticsModuleType.CTREPCM, 6, 7);

    }
    
    public void setClimbUp() {
        climberSolenoid.set(DoubleSolenoid.Value.kForward);
        m_climbed = true;
    }

    public void setClimbDown() {
        climberSolenoid.set(DoubleSolenoid.Value.kReverse);
        m_climbed = false;
    }

     public void setClimbStop() {
        climberSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    public Command climbStopCommand () {
        return new InstantCommand(() -> setClimbStop(), this);
    }

    public Command climbDownCommand(double time) {
        return Commands.sequence(
            new InstantCommand(() -> {
                if (m_climbed) {
                    setClimbDown();
                }
            }, this),
            Commands.waitSeconds(time),
            climbStopCommand()
        );
    }

    public Command climbUpCommand(double time) {
      return Commands.sequence(
            new InstantCommand(() -> {
                if (!m_climbed) {
                    setClimbUp();
                }
            }, this),
            Commands.waitSeconds(time),
            climbStopCommand()
        ); 
    }

}