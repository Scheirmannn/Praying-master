package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;


public class PneumaticSubsystem extends SubsystemBase {
    
    private final Compressor compressor;

    public PneumaticSubsystem() {
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();
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
}
