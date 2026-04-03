package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

  public final AHRS m_gyro = new AHRS(NavXComType.kUSB1);

  private double m_simGyroAngle = 0.0;
  private ChassisSpeeds m_lastChassisSpeeds = new ChassisSpeeds();

  private final SwerveDrivePoseEstimator m_poseEstimator;

  private final PIDController m_xController = new PIDController(11.0, 0, 0.1);
  private final PIDController m_yController = new PIDController(11.0, 0, 0.1);
  private final PIDController m_rotController = new PIDController(10.0, 0, 0.2);

  private final PIDController m_visionAlignController = new PIDController(0.04, 0, 0.001);

  private final Field2d m_field = new Field2d();

  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    m_gyro.reset();

    m_rotController.enableContinuousInput(-Math.PI, Math.PI);
    m_visionAlignController.setTolerance(1.0);

    m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        new Pose2d(),
        VecBuilder.fill(1, 1, 1),
        VecBuilder.fill(0.01, 0.01, 0.01));

    SmartDashboard.putData("Field", m_field);
  }

  private Rotation2d getGyroRotation() {
    if (RobotBase.isSimulation()) {
      return Rotation2d.fromDegrees(m_simGyroAngle);
    }
    return m_gyro.getRotation2d();
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    Pose2d pose = getPose();
    m_field.setRobotPose(pose);
  }

  @Override
  public void simulationPeriodic() {
    m_frontLeft.simulationPeriodic();
    m_frontRight.simulationPeriodic();
    m_rearLeft.simulationPeriodic();
    m_rearRight.simulationPeriodic();

    double omegaRadPerSec = m_lastChassisSpeeds.omegaRadiansPerSecond;
    m_simGyroAngle += Math.toDegrees(omegaRadPerSec * 0.02);

    while (m_simGyroAngle > 180)
      m_simGyroAngle -= 360;
    while (m_simGyroAngle < -180)
      m_simGyroAngle += 360;

    // Update field with sim pose so the robot moves on the dashboard in sim
    m_field.setRobotPose(getPose());
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);

    // Reset field pose immediately so dashboard doesn't show stale position
    m_field.setRobotPose(pose);
  }

  public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    m_poseEstimator.addVisionMeasurement(pose, timestampSeconds, stdDevs);
    m_field.getObject("vision").setPose(pose);
  }

  public void followTrajectory(SwerveSample sample) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      sample = sample.flipped();
    }

    Pose2d pose = getPose();
  
    SmartDashboard.putNumberArray("Choreo Target Pose", new double[] {
        sample.x, sample.y, sample.heading
    });

    SmartDashboard.putNumber("Choreo/XError", sample.x - pose.getX());
    SmartDashboard.putNumber("Choreo/YError", sample.y - pose.getY());
    SmartDashboard.putNumber("Choreo/RotError", sample.heading - pose.getRotation().getRadians());

    double rotOutput = m_rotController.calculate(pose.getRotation().getRadians(), sample.heading);
    SmartDashboard.putNumber("Choreo/RotOutput", rotOutput);

    ChassisSpeeds speeds = new ChassisSpeeds(
        sample.vx + m_xController.calculate(pose.getX(), sample.x),
        sample.vy + m_yController.calculate(pose.getY(), sample.y),
        sample.omega + rotOutput);

    var chassisField = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.getRotation());
    m_lastChassisSpeeds = chassisField;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisField);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    SmartDashboard.putNumber("Choreo/TargetOmegaRadPerSec", sample.omega);

    if (Math.abs(sample.vx) < 0.01 && Math.abs(sample.vy) < 0.01 && Math.abs(sample.omega) < 0.01) {
      m_lastChassisSpeeds = new ChassisSpeeds();
    }
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered, getGyroRotation())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void alignToTarget(double yawErrorDegrees) {
    double rot = m_visionAlignController.calculate(yawErrorDegrees, 0);
    drive(0, 0, rot, false);
    SmartDashboard.putNumber("Vision/AlignOutput", rot);
  }

  public boolean isAligned() {
    return m_visionAlignController.atSetpoint();
  }

  public Command alignToTagCommand(Vision vision) {
    return new RunCommand(() -> {
      var yaw = vision.getYawToAllianceTarget();
      if (yaw.isPresent()) {
        alignToTarget(yaw.getAsDouble());
      } else {
        stopModules();
      }
    }, this).until(this::isAligned);
  }

  public void stopModules() {
    m_lastChassisSpeeds = new ChassisSpeeds();
    m_frontLeft.setDesiredState(new SwerveModuleState(0, m_frontLeft.getState().angle));
    m_frontRight.setDesiredState(new SwerveModuleState(0, m_frontRight.getState().angle));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, m_rearLeft.getState().angle));
    m_rearRight.setDesiredState(new SwerveModuleState(0, m_rearRight.getState().angle));
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
  }

  public double getHeading() {
    return getGyroRotation().getDegrees();
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

  public Field2d getField() {
    return m_field;
  }

  public void log() {
    Pose2d pose = getPose();

    SmartDashboard.putNumberArray("Robot Pose", new double[] {
        pose.getX(),
        pose.getY(),
        pose.getRotation().getRadians()
    });

    SmartDashboard.putNumberArray("Swerve States", new double[] {
        m_frontLeft.getState().angle.getRadians(),
        m_frontLeft.getState().speedMetersPerSecond,
        m_frontRight.getState().angle.getRadians(),
        m_frontRight.getState().speedMetersPerSecond,
        m_rearLeft.getState().angle.getRadians(),
        m_rearLeft.getState().speedMetersPerSecond,
        m_rearRight.getState().angle.getRadians(),
        m_rearRight.getState().speedMetersPerSecond
    });
  }
}