package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;


public class PneumaticSubsystem extends SubsystemBase {
    
    private final Compressor compressor;

    public PneumaticSubsystem() {
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();

        SmartDashboard.putBoolean("Is at Pressure", false);

    }

    public boolean isAtPressure() {
        return compressor.getPressureSwitchValue();
    }

    public void enableCompressor() {
        compressor.enableDigital();
    }

    public void disableCompressor() {
        compressor.disable();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Is at Pressure",isAtPressure());
    }
}
