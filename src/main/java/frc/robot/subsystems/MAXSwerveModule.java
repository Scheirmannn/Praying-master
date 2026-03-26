package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Configs;

public class MAXSwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;
  private SparkSim m_drivingSparkSim;
  private SparkSim m_turningSparkSim;
  private double m_simDrivePosition = 0;
  private double m_simTurningPosition = 0;
  private double m_simDriveVelocity = 0;
  private static final double kTurnSimP = 20.0;
  private static final double kSimUpdatePeriod = .02;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);

   if (RobotBase.isSimulation()) {
      m_drivingSparkSim = new SparkSim(m_drivingSpark, DCMotor.getNEO(1));
      m_turningSparkSim = new SparkSim(m_turningSpark, DCMotor.getNeo550(1));

      m_simTurningPosition = chassisAngularOffset;
      m_simDrivePosition = 0;
      m_simDriveVelocity = 0;

    }
  }

  public void simulationPeriodic() {
    if (!RobotBase.isSimulation()) {
      return;
    }

    // Simulate driving motor
    m_drivingSparkSim.iterate(m_drivingSpark.getAppliedOutput(), 12.0, 0.02);

    // Update drive velocity from simulation
    m_simDriveVelocity = m_drivingSparkSim.getVelocity();

    // Update drive position by integrating velocity
    m_simDrivePosition += m_simDriveVelocity * 0.16;

    // CRITICAL FIX: Simulate turning motor
    m_turningSparkSim.iterate(m_turningSpark.getAppliedOutput(), 12.0, 0.02);

    // Get desired turn position from the desired state
    double desiredTurnPosition = m_desiredState.angle.getRadians() + m_chassisAngularOffset;
    double currentTurnPosition = m_simTurningPosition;

    // Calculate shortest path to desired angle
    double turnError = desiredTurnPosition - currentTurnPosition;
    while (turnError > Math.PI)
      turnError -= 2 * Math.PI;
    while (turnError < -Math.PI)
      turnError += 2 * Math.PI;

    // Simple proportional control for turning (faster response)
    double turnVelocity = turnError * kTurnSimP;
    m_simTurningPosition += turnVelocity * kSimUpdatePeriod;

    // Normalize position to [0, 2*PI]
    while (m_simTurningPosition > 2 * Math.PI)
      m_simTurningPosition -= 2 * Math.PI;
    while (m_simTurningPosition < 0)
      m_simTurningPosition += 2 * Math.PI;
  }

  public SwerveModuleState getState() {
    if (RobotBase.isSimulation()) {
        return new SwerveModuleState(
            m_simDriveVelocity,
            new Rotation2d(m_simTurningPosition - m_chassisAngularOffset));
    }
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public SwerveModulePosition getPosition() {
    if (RobotBase.isSimulation()) {
        return new SwerveModulePosition(
            m_simDrivePosition,
            new Rotation2d(m_simTurningPosition - m_chassisAngularOffset));
    }
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    correctedDesiredState.optimize(RobotBase.isSimulation()
        ? new Rotation2d(m_simTurningPosition)
        : new Rotation2d(m_turningEncoder.getPosition()));

    m_drivingClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    if (RobotBase.isSimulation()) {
        m_simDriveVelocity = correctedDesiredState.speedMetersPerSecond;
        m_simTurningPosition = correctedDesiredState.angle.getRadians();
        m_simDrivePosition += m_simDriveVelocity * 0.02;
    }

    m_desiredState = desiredState;
  }

  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
    if (RobotBase.isSimulation()) {
        m_simDrivePosition = 0.0;
        m_simDriveVelocity = 0.0;
    }
  }
}