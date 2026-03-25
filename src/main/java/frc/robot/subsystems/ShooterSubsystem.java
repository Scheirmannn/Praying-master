//everything for the shooter prob
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Constants.UtilityConstants;
import frc.robot.Configs;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private final SparkMax gateMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    
    public enum speedProfiles {
        LOW(15.0),
        MID(17.0),
        HIGH(20.0),
        MAX(28.0);


        public final double speed;

        speedProfiles(double speed) {
            this.speed = speed;
        }

    }
    
    public enum gateProfiles {
        LOW(0.50),
        MEDIUM(0.67),
        HIGH(0.83);

        public final double gatePower;

        gateProfiles(double gatePower) {
            this.gatePower = gatePower;
        }

    }

    private speedProfiles m_currentSpeed = speedProfiles.LOW;
    private gateProfiles m_currentGatePower = gateProfiles.LOW;

    public ShooterSubsystem(int leftMotorId, int rightMotorId, int gateMotorId) {

        leftMotor = new SparkMax(leftMotorId, SparkMax.MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorId, SparkMax.MotorType.kBrushless);

        gateMotor = new SparkMax(gateMotorId, SparkMax.MotorType.kBrushless);
        
        rightMotor.configure(Configs.Utility.vortexConfig, ResetMode.kResetSafeParameters,
        
        PersistMode.kPersistParameters);

        Configs.Utility.vortexConfig.inverted(true);

        leftMotor.configure(Configs.Utility.vortexConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        Configs.Utility.vortexConfig.inverted(false);

        gateMotor.configure(Configs.Utility.neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        SmartDashboard.putString("Current Speed", m_currentSpeed.name());
        SmartDashboard.putNumber("Current Gate Power", m_currentGatePower.gatePower);
        SmartDashboard.putBoolean("Shooter at Speed", false);

    }

    
    //   cycling methods + vars

   
    public void cycleSpeeds() {
        speedProfiles[] profile = speedProfiles.values();
        int next = (m_currentSpeed.ordinal() + 1) % profile.length;
        m_currentSpeed = profile[next];
        SmartDashboard.putString("Current Speed", m_currentSpeed.name());
    }

    public void cycleGatePowers() {
        gateProfiles[] profiles = gateProfiles.values();
        int next = (m_currentGatePower.ordinal() + 1) % profiles.length;
        m_currentGatePower = profiles[next];
        SmartDashboard.putNumber("Current Gate Power", m_currentGatePower.gatePower);

    }

    public void setSpeedProfile(speedProfiles profile) {
        m_currentSpeed = profile;
        SmartDashboard.putString("Current Speed", m_currentSpeed.name());
    }

    public double currentSpeed() {
        return m_currentSpeed.speed;
    }

    public double currentGatePower() {
        return m_currentGatePower.gatePower;
    }

    //   Other vars

    public double getShooterVelocity() {
        double avgRPM = (Math.abs(leftEncoder.getVelocity()) + Math.abs(rightEncoder.getVelocity())) / 2;
        double radPerSec = avgRPM * 2 * Math.PI / 60;
        return (Math.round(radPerSec * UtilityConstants.kShooterRadius *2) / (2.0));

    }
    
    public boolean isAtSpeed() {
        return getShooterVelocity() >= (currentSpeed() * 0.95);

    }

    public boolean isShooterRunning() {
        return getShooterVelocity() > 1;

    }

    //   Hardware methods

    public void shooterStop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public void setShooterVelocity(double metersPerSec) {
        double radPerSec = metersPerSec / UtilityConstants.kShooterRadius;
        double rpm = radPerSec * 60 / (2 * Math.PI);
        leftMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
        rightMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    }

    public void setGatePower(double power) {
        gateMotor.set(power);

    }

    public void gateStop() {
        gateMotor.stopMotor();
    }

    //   Basic Commands

    public Command shooterSpinUpCommand() {
        return new RunCommand(() -> setShooterVelocity(currentSpeed()));
    }

    public Command gateStartCommand() {
        return new InstantCommand(() -> setGatePower(currentGatePower()), this);

    }

    public Command gateReverseCommand(double gatePower) {
        return new InstantCommand(() -> setGatePower(-gatePower), this);

    }

    public Command gateWaitCommand() {
        return Commands.sequence(
            Commands.waitUntil(this::isAtSpeed),
            Commands.waitSeconds(0.5),
            gateStartCommand()
        
        );
    }

    public Command gateStopCommand() {
        return new InstantCommand(() -> {
            setGatePower(0);
        }, this);
    }

    public Command shooterStopCommand() {
        return new InstantCommand(() -> {
            setShooterVelocity(0);
        }, this);
    }
    
    public Command dualStopCommand() {
        return new InstantCommand (() ->{
            gateStop();
            shooterStop();
        }, this);
    }


    public Command fullShootCommand() {
        return Commands.parallel(
            shooterSpinUpCommand(),
            gateWaitCommand()
        );
    }

    public Command cycleShootSpeedCommand() {
        return new InstantCommand(() -> {
            cycleSpeeds();
        }, this);
    }

    public Command cycleGatePowerCommand() {
        return new InstantCommand(() -> {
            cycleGatePowers();
        }, this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter at Speed", isAtSpeed());
        SmartDashboard.putNumber("Shooter Speed (MetersPerSec)", getShooterVelocity());
        SmartDashboard.putNumber("Ideal Speed", currentSpeed());
    }

}


