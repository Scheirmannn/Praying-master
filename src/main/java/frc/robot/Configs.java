package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.UtilityConstants;

public final class Configs {
        public static final class Utility {
                public static final SparkMaxConfig vortexConfig = new SparkMaxConfig();
                public static final SparkMaxConfig neoConfig = new SparkMaxConfig();
                
                static {
                double nominalVoltage = 12.0;
                double vortexVelocityFeedForward = nominalVoltage/ UtilityConstants.kVortexFreeSpeedRpm;
                double neoVelocityFeedForward = nominalVoltage/ UtilityConstants.kNeoFreeSpeedRps;

                vortexConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(40)
                        .closedLoopRampRate(1.0);
                vortexConfig.encoder
                        .positionConversionFactor(1)
                        .velocityConversionFactor(1);
                vortexConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                       // no idea what to put for pid chat said ts tho
                        .pid(0.0002, 0.0, 0.0)
                        .outputRange(-1, 1)
                        .feedForward.kV(vortexVelocityFeedForward);
                
                neoConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(40);
                neoConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(0.0002,0.0,0.0)
                        .outputRange(-1, 1)
                        .feedForward.kV(neoVelocityFeedForward);
                }



        }
        public static final class MAXSwerveModule {
                public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
                public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
                double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
                double turningFactor = 2 * Math.PI;
                double nominalVoltage = 12.0;
                double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

                drivingConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(50);
                drivingConfig.encoder
                        .positionConversionFactor(drivingFactor) // meters
                        .velocityConversionFactor(drivingFactor / 60.0); // meters per second
                drivingConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(0.04, 0, 0)
                        .outputRange(-1, 1)
                        .feedForward.kV(drivingVelocityFeedForward);
            

                turningConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(20);
                turningConfig.absoluteEncoder
                        // Invert the turning encoder, since the output shaft rotates in the opposite
                        // direction of the steering motor in the MAXSwerve Module.
                         .inverted(true)
                        .positionConversionFactor(turningFactor) // radians
                        .velocityConversionFactor(turningFactor / 60.0) // radians per second
                        // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
                        .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);
                turningConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(1, 0, 0)
                        .outputRange(-1, 1)
                        // Enable PID wrap around for the turning motor. This will allow the PID
                        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                        // to 10 degrees will go through 0 rather than the other direction which is a
                        // longer route.
                        .positionWrappingEnabled(true)
                        .positionWrappingInputRange(0, turningFactor);
        }
    }
}
