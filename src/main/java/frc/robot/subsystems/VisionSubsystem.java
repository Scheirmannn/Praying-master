package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.simulation.PhotonCameraSim;
import edu.wpi.first.math.controller.PIDController;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.VecBuilder;
import org.photonvision.PhotonCamera;
import java.util.function.BiConsumer;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final BiConsumer<Pose2d, Double> poseConsumer;

  private PhotonTrackedTarget bestTarget;
  private boolean hasTargets;
  private double targetYaw;
  private double targetPitch;
  private double targetArea;

  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;

  public VisionSubsystem(BiConsumer<Pose2d, Double> poseConsumer) {
    aprilTagFieldLayout = AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();
    camera = new PhotonCamera("OV5647");
    this.poseConsumer = poseConsumer;
    photonEstimator = new PhotonPoseEstimator(
        aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        Constants.Vision.kRobotToCam
    );
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagFieldLayout);
      
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(.35, .10);
      cameraProp.setFPS(60);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      
      cameraSim = new PhotonCameraSim(camera, cameraProp);
      visionSim.addCamera(cameraSim, Constants.Vision.kRobotToCam);
      cameraSim.enableDrawWireframe(true);
    }
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose();

    if (estimatedPose.isPresent() && !RobotBase.isSimulation()) {
        EstimatedRobotPose pose = estimatedPose.get();
        poseConsumer.accept(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    }
    
    PhotonPipelineResult result = camera.getLatestResult();

    hasTargets = result.hasTargets();

    if (hasTargets) {
      bestTarget = result.getBestTarget();
      targetYaw = bestTarget.getYaw();
      targetPitch = bestTarget.getPitch();
      targetArea = bestTarget.getArea();
    } else {
      bestTarget = null;
      targetYaw = 0;
      targetPitch = 0;
      targetArea = 0;
    }

    SmartDashboard.putBoolean("Vision/Has Targets", hasTargets);
    SmartDashboard.putNumber("Vision/Target Yaw", targetYaw);
    SmartDashboard.putNumber("Vision/Target Pitch", targetPitch);
    SmartDashboard.putNumber("Vision/Target Area", targetArea);
    SmartDashboard.putNumber("Vision/Target ID", getTargetId());


    if (estimatedPose.isPresent()) {
      EstimatedRobotPose pose = estimatedPose.get();

      SmartDashboard.putNumber("Vision Pose X", pose.estimatedPose.toPose2d().getX());
      SmartDashboard.putNumber("Vision Pose Y", pose.estimatedPose.toPose2d().getY());
      SmartDashboard.putNumber("Vision Timestamp", pose.timestampSeconds);

      var stdDevs = getEstimationStdDevs(pose);

      poseConsumer.accept(pose.estimatedPose.toPose2d(), pose.timestampSeconds);

      SmartDashboard.putString("Vision Std Devs", stdDevs);
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var result = camera.getLatestResult();
    if (!result.hasTargets()) {
      return Optional.empty();
    }
    return photonEstimator.update(result);
  }

  private String getEstimationStdDevs(EstimatedRobotPose estimatedPose) {
    var estStdDevs = Constants.Vision.kSingleTagStdDevs;
    int numTags = estimatedPose.targetsUsed.size();
    double avgDist = 0;

    for (var target : estimatedPose.targetsUsed) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(target.getFiducialId());
      if (tagPose.isPresent()) {
        avgDist += tagPose.get().toPose2d().getTranslation()
            .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
      }
    }

    if (numTags > 0) {
      avgDist /= numTags;
    }

    if (numTags > 1) {
      estStdDevs = Constants.Vision.kMultiTagStdDevs;
    }

    if (numTags == 1 && avgDist > 4) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    }

    return String.format("Tags: %d, Dist: %.2f m, StdDevs: [%.2f, %.2f, %.2f]",
        numTags, avgDist, estStdDevs.get(0, 0), estStdDevs.get(1, 0), estStdDevs.get(2, 0));
  }

  public boolean hasTargets() {
    return hasTargets;
  }

  public double getTargetYaw() {
    return targetYaw;
  }

  public double getTargetPitch() {
    return targetPitch;
  }

  public double getTargetArea() {
    return targetArea;
  }

  public int getTargetId() {
    return bestTarget != null ? bestTarget.getFiducialId() : -1;
  }

  public PhotonTrackedTarget getBestTarget() {
    return bestTarget;
  }

  public PhotonCamera getCamera() {
    return camera;
  }

  public void simulationPeriodic(Pose2d robotSimPose) {
    if (RobotBase.isSimulation()) {
      visionSim.update(robotSimPose);
    }
  }

  public void resetSimPose(Pose2d pose) {
    if (RobotBase.isSimulation()) {
      visionSim.resetRobotPose(pose);
    }
  }

  public Command getAlignCommand(DriveSubsystem drive) {
    return new Command() {
      private final PIDController rotController = new PIDController(0.05, 0, 0.01);

      {
        rotController.setTolerance(2.0);
        addRequirements(drive);
      }

      @Override
      public void initialize() {
        rotController.reset();
      }

      @Override
      public void execute() {
        if (hasTargets()) {
          double rot = rotController.calculate(getTargetYaw(), 0);
          rot = MathUtil.clamp(rot, -1, 1);
          drive.drive(0, 0, rot, false);
        }
      }

      @Override
      public boolean isFinished() {
        return hasTargets() && rotController.atSetpoint();
      }

      @Override
      public void end(boolean interrupted) {
        drive.drive(0, 0, 0, false);
      }
    };
  }
}