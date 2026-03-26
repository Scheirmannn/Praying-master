package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SparkConstants;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;

public class DriveSubsystem extends SubsystemBase {

  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      SparkConstants.kFrontLeftDrivingCanId,
      SparkConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      SparkConstants.kFrontRightDrivingCanId,
      SparkConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      SparkConstants.kRearLeftDrivingCanId,
      SparkConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      SparkConstants.kRearRightDrivingCanId,
      SparkConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  public final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  // Sim
  private double m_simGyroAngle = 0.0; // Sim gyro angle
  private ChassisSpeeds m_lastChassisSpeeds = new ChassisSpeeds(); // Track last commanded speeds

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  @Override
  public void simulationPeriodic() {
    m_frontLeft.simulationPeriodic();
    m_frontRight.simulationPeriodic();
    m_rearLeft.simulationPeriodic();
    m_rearRight.simulationPeriodic();

    double omegaRadPerSec = m_lastChassisSpeeds.omegaRadiansPerSecond;

    m_simGyroAngle += Math.toDegrees(omegaRadPerSec * .02);

    while (m_simGyroAngle > 180)
      m_simGyroAngle -= 360;
    while (m_simGyroAngle < -180)
      m_simGyroAngle += 360;
  }

  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    // Wait for gyro to calibrate
    m_gyro.reset();
  }

  public void log() {
        // Robot pose (position on the field, if odometry is set up)
        Pose2d pose = getPose();

        //Add pose array for field visualization
        SmartDashboard.putNumberArray("Robot Pose", new double[] {
          pose.getX(),
          pose.getY(),
          pose.getRotation().getRadians()});
          
        SmartDashboard.putNumberArray("Swerve States", new double[] {
          m_frontLeft.getState().angle.getRadians(),
          m_frontLeft.getState().speedMetersPerSecond,
          m_frontRight.getState().angle.getRadians(),
          m_frontRight.getState().speedMetersPerSecond,
          m_rearLeft.getState().angle.getRadians(),
          m_rearLeft.getState().speedMetersPerSecond,
          m_rearRight.getState().angle.getRadians(),
          m_rearRight.getState().speedMetersPerSecond});      
    }

  @Override
  public void periodic() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  private final PIDController m_xController = new PIDController(11.0, 0, 0.1);
  private final PIDController m_yController = new PIDController(11.0, 0, 0.1);
  private final PIDController m_rotController = new PIDController(4.0, 0, 0.2); 

  public void followTrajectory(SwerveSample sample) {
    // Flip sample if on Red alliance
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        sample = sample.flipped();
    }

    SmartDashboard.putNumberArray("Choreo Target Pose", new double[] {
        sample.x,
        sample.y,
        sample.heading
    });

    Pose2d pose = getPose();

    ChassisSpeeds speeds = new ChassisSpeeds(
        sample.vx + m_xController.calculate(pose.getX(), sample.x),
        sample.vy + m_yController.calculate(pose.getY(), sample.y),
        sample.omega + m_rotController.calculate(pose.getRotation().getRadians(), sample.heading));

    var chassisField = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.getRotation());
    m_lastChassisSpeeds = chassisField;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisField);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    SmartDashboard.putNumber("Choreo/TargetOmegaRadPerSec", sample.omega);
  }
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                m_gyro.getRotation2d().plus(Rotation2d.fromDegrees(180)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setSimStartingPose(Pose2d pose) {
    if (RobotBase.isSimulation()) {
        m_simGyroAngle = pose.getRotation().getDegrees();
        resetOdometry(pose);
    }
  }

}