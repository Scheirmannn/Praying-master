// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SparkConstants;

public class DriveSubsystem extends SubsystemBase {
  public static double kSimVxMPS = 0.0;
  public static double kSimVyMPS = 0.0;
  // Only sim field needed — gyro angle for field-relative driving
  private static double kSimGyroAngleRad = 0.0;

  public static void setSimGyroAngleRad(double value) {
      if (value != kSimGyroAngleRad) {
          System.out.println("gyro changed from " + kSimGyroAngleRad + " to " + value);
          Thread.dumpStack();
      }
      kSimGyroAngleRad = value;
  }
  
  public static double getSimGyroAngleRad() {
    return kSimGyroAngleRad;
  }

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

  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(this::addVisionMeasurement);

  private final SwerveDrivePoseEstimator m_poseEstimator;

  public DriveSubsystem() {
    Rotation2d initialGyro = RobotBase.isSimulation()
      ? new Rotation2d(0)
      : Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ));

    m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,         
        initialGyro,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        new Pose2d(0, 0, new Rotation2d(0)),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    );

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  @Override
  public void periodic() {
    
    Rotation2d gyroAngle = RobotBase.isSimulation()
        ? new Rotation2d(getSimGyroAngleRad())
        : Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ));

    m_poseEstimator.update(gyroAngle,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }


  private Pose2d m_simPose = new Pose2d();

  public Pose2d getPose() {
      if (RobotBase.isSimulation()) {
          return m_simPose;
      }
      return m_poseEstimator.getEstimatedPosition();
  }

  public void updateSimPose(Pose2d pose) {
      m_simPose = pose;
  }

  public void resetOdometry(Pose2d pose) {
    Rotation2d gyroAngle = RobotBase.isSimulation()
        ? new Rotation2d(getSimGyroAngleRad())
        : Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ));

    m_poseEstimator.resetPosition(
        gyroAngle,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);


  }
  private Rotation2d getGyroAngle() {
    return RobotBase.isSimulation()
        ? new Rotation2d(getSimGyroAngleRad())
        : Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ));
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    Rotation2d gyroAngle = getGyroAngle();

    ChassisSpeeds speeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, gyroAngle)
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    kSimVxMPS = speeds.vxMetersPerSecond;
    kSimVyMPS = speeds.vyMetersPerSecond;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    // Update gyro AFTER everything else
    if (RobotBase.isSimulation()) {
        setSimGyroAngleRad(getSimGyroAngleRad() + rotDelivered * 0.02);
    }
  } 

  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
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
    if (RobotBase.isSimulation()) {
      kSimGyroAngleRad = 0.0;
    }
  }

  public double getHeading() {
       return RobotBase.isSimulation()
        ? Math.toDegrees(getSimGyroAngleRad())
        : Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }

  public double getTurnRate() {
    return RobotBase.isSimulation()
        ? 0.0
        : m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  private void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    m_poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  public VisionSubsystem getVisionSubsystem() {
    return m_VisionSubsystem;
  }
}