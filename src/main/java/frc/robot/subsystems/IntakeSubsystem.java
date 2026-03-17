//everything for the shooter prob
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Configs;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax rightMotor;
    private final SparkMax leftMotor;
    private final DoubleSolenoid intakeSolenoid;

    public boolean m_intakeUp = true;

    public IntakeSubsystem(int leftMotorId, int rightMotorId) {

        leftMotor = new SparkMax(leftMotorId, SparkMax.MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorId, SparkMax.MotorType.kBrushless);
        intakeSolenoid = new DoubleSolenoid(13, PneumaticsModuleType.CTREPCM, 0, 1);

        leftMotor.configure(Configs.Utility.neoConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        rightMotor.configure(Configs.Utility.neoConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        Configs.Utility.neoConfig.inverted(false);
        SmartDashboard.putBoolean("Intake arm up", false);
    
    }
    

    public void setArmDown() {
        intakeSolenoid.set(DoubleSolenoid.Value.kForward);
        m_intakeUp = false;
        SmartDashboard.putBoolean("Intake arm up", m_intakeUp);
    }

     public void setArmUp() {
        intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
        m_intakeUp = true;
        SmartDashboard.putBoolean("Intake arm up", m_intakeUp);
    }

     public void setArmStop() {
        intakeSolenoid.set(DoubleSolenoid.Value.kOff);
    }
    

    public void setIntakeVelocity(double power) {
        leftMotor.set(power);
        rightMotor.set(power);
    }
    
    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();

    }

    public Command intakeArmStopCommand () {
        return new InstantCommand(() -> setArmStop(), this);
    }

    public Command IntakeArmDownCommand(double time) {
        return Commands.sequence(
            new InstantCommand(() -> {
                setArmDown();
            }, this),
            Commands.waitSeconds(time),
            intakeArmStopCommand()
        );
    }

    public Command IntakeArmUpCommand(double time) {
      return Commands.sequence(
            new InstantCommand(() -> {
                if (!m_intakeUp) {
                    setArmUp();
                }
            }, this),
            Commands.waitSeconds(time),
            intakeArmStopCommand()
        ); 
    }


    public Command stopIntakeCommand() {
        return new InstantCommand(() -> stop(), this);
    }

    
    public Command setIntakeCommand(double power) {
        return new RunCommand(() -> {
            setIntakeVelocity(power);
        }, this);
    }

    public Command fullIntakeCommand(double power, double time) {
        return Commands.sequence(
            IntakeArmDownCommand(time),
            setIntakeCommand(power)

        );
    }



}