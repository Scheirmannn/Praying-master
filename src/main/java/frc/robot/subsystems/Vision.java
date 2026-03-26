package frc.robot.subsystems;

import java.util.Optional;
import java.util.OptionalDouble;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Vision extends SubsystemBase {

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
    }

    private final PhotonCamera frontLeftCamera;
    private final PhotonCamera frontRightCamera;

    private final PhotonPoseEstimator frontLeftEstimator;
    private final PhotonPoseEstimator frontRightEstimator;

    private VisionSystemSim visionSim;
    private PhotonCameraSim frontLeftCamSim;
    private PhotonCameraSim frontRightCamSim;

    private final VisionConsumer poseConsumer;

    public Vision(VisionConsumer poseConsumer) {
        this.poseConsumer = poseConsumer;

        frontLeftCamera = new PhotonCamera(Constants.Vision.kFrontLeftCameraName);
        frontRightCamera = new PhotonCamera(Constants.Vision.kFrontRightCameraName);

        frontLeftEstimator = new PhotonPoseEstimator(
                Constants.Vision.kTagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.Vision.kRobotToFrontLeftCamera);
        frontLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        frontRightEstimator = new PhotonPoseEstimator(
                Constants.Vision.kTagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.Vision.kRobotToFrontRightCamera);
        frontRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(Constants.Vision.kTagLayout);

            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(60);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);

            frontLeftCamSim = new PhotonCameraSim(frontLeftCamera, cameraProp);
            visionSim.addCamera(frontLeftCamSim, Constants.Vision.kRobotToFrontLeftCamera);
            frontLeftCamSim.enableDrawWireframe(true);

            frontRightCamSim = new PhotonCameraSim(frontRightCamera, cameraProp);
            visionSim.addCamera(frontRightCamSim, Constants.Vision.kRobotToFrontRightCamera);
            frontRightCamSim.enableDrawWireframe(true);
        }
    }

    @Override
    public void periodic() {
        processCameraResult(frontLeftCamera, frontLeftEstimator,
                Constants.Vision.kFrontLeftCameraYawDegrees, "FL");
        processCameraResult(frontRightCamera, frontRightEstimator,
                Constants.Vision.kFrontRightCameraYawDegrees, "FR");
    }

    public int[] getTargetTagIds() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return Constants.Vision.kRedAllianceTagIds;
        }
        return Constants.Vision.kBlueAllianceTagIds;
    }

    public boolean hasTarget() {
        int[] targetIds = getTargetTagIds();
        for (PhotonCamera camera : new PhotonCamera[] { frontLeftCamera, frontRightCamera }) {
            PhotonPipelineResult result = camera.getLatestResult();
            if (!result.hasTargets())
                continue;
            for (var target : result.getTargets()) {
                for (int id : targetIds) {
                    if (target.getFiducialId() == id)
                        return true;
                }
            }
        }
        return false;
    }

    private void processCameraResult(PhotonCamera camera, PhotonPoseEstimator estimator,
            double cameraYawOffsetDegrees, String prefix) {

        PhotonPipelineResult result = camera.getLatestResult();
        SmartDashboard.putBoolean("Vision/" + prefix + "/Connected", camera.isConnected());
        SmartDashboard.putBoolean("Vision/" + prefix + "/Has Targets", result.hasTargets());

        if (result.hasTargets()) {
            SmartDashboard.putNumber("Vision/" + prefix + "/Target Count",
                    result.getTargets().size());
            SmartDashboard.putNumber("Vision/" + prefix + "/Best Tag ID",
                    result.getBestTarget().getFiducialId());
        }

        if (!result.hasTargets())
            return;

        Optional<EstimatedRobotPose> estimatedPose = estimator.update(result);
        if (estimatedPose.isEmpty())
            return;

        EstimatedRobotPose pose = estimatedPose.get();
        Pose2d pose2d = pose.estimatedPose.toPose2d();

        SmartDashboard.putNumber("Vision/" + prefix + "/Pose X", pose2d.getX());
        SmartDashboard.putNumber("Vision/" + prefix + "/Pose Y", pose2d.getY());
        SmartDashboard.putNumber("Vision/" + prefix + "/Timestamp", pose.timestampSeconds);

        Matrix<N3, N1> stdDevs = computeStdDevs(pose, estimator);

        SmartDashboard.putString("Vision/" + prefix + "/Std Devs",
                String.format("[%.2f, %.2f, %.2f]",
                        stdDevs.get(0, 0), stdDevs.get(1, 0), stdDevs.get(2, 0)));

        poseConsumer.accept(pose2d, pose.timestampSeconds, stdDevs);
    }

    private Matrix<N3, N1> computeStdDevs(EstimatedRobotPose estimatedPose,
            PhotonPoseEstimator estimator) {

        Matrix<N3, N1> estStdDevs = Constants.Vision.kSingleTagStdDevs;
        int numTags = estimatedPose.targetsUsed.size();
        double avgDist = 0;

        for (var target : estimatedPose.targetsUsed) {
            var tagPose = estimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                avgDist += tagPose.get().toPose2d().getTranslation()
                        .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
            }
        }

        if (numTags > 0)
            avgDist /= numTags;

        if (numTags > 1) {
            estStdDevs = Constants.Vision.kMultiTagStdDevs;
        }

        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        return estStdDevs;
    }

    public OptionalDouble getYawToAllianceTarget() {
        OptionalDouble leftYaw = getYawFromCamera(frontLeftCamera,
                Constants.Vision.kFrontLeftCameraYawDegrees);
        OptionalDouble rightYaw = getYawFromCamera(frontRightCamera,
                Constants.Vision.kFrontRightCameraYawDegrees);

        if (leftYaw.isPresent() && rightYaw.isPresent()) {
            return Math.abs(leftYaw.getAsDouble()) < Math.abs(rightYaw.getAsDouble())
                    ? leftYaw
                    : rightYaw;
        }
        if (leftYaw.isPresent())
            return leftYaw;
        if (rightYaw.isPresent())
            return rightYaw;
        return OptionalDouble.empty();
    }

    private OptionalDouble getYawFromCamera(PhotonCamera camera, double cameraYawOffsetDegrees) {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets())
            return OptionalDouble.empty();

        int[] targetIds = getTargetTagIds();
        for (var target : result.getTargets()) {
            for (int id : targetIds) {
                if (target.getFiducialId() == id) {
                    double correctedYaw = target.getYaw() + cameraYawOffsetDegrees;
                    SmartDashboard.putNumber("Vision/RawYaw/" + camera.getName(), target.getYaw());
                    SmartDashboard.putNumber("Vision/CorrectedYaw/" + camera.getName(), correctedYaw);
                    return OptionalDouble.of(correctedYaw);
                }
            }
        }
        return OptionalDouble.empty();
    }

    public double getHubDistance() {
        record CameraEntry(PhotonCamera cam, double lateralOffsetMeters) {
        }

        var cameras = new CameraEntry[] {
                new CameraEntry(frontLeftCamera, Constants.Vision.kFrontLeftCameraLateralOffsetMeters),
                new CameraEntry(frontRightCamera, Constants.Vision.kFrontRightCameraLateralOffsetMeters)
        };

        int[] targetIds = getTargetTagIds();

        for (var entry : cameras) {
            PhotonPipelineResult result = entry.cam().getLatestResult();
            if (!result.hasTargets())
                continue;

            for (var target : result.getTargets()) {
                for (int id : targetIds) {
                    if (target.getFiducialId() == id) {
                        double camDist = target.getBestCameraToTarget().getTranslation().getNorm();
                        double offsetMeters = Math.abs(entry.lateralOffsetMeters());
                        double angleRad = Math.toRadians(target.getYaw());

                        double robotDist = Math.sqrt(
                                (camDist * camDist)
                                        + (offsetMeters * offsetMeters)
                                        - 2 * camDist * offsetMeters * Math.cos(Math.PI / 2 - angleRad));

                        SmartDashboard.putNumber("Shooter/Distance (m)", robotDist);
                        return robotDist;
                    }
                }
            }
        }

        SmartDashboard.putNumber("Shooter/Distance (m)", -1);
        return -1;
    }

    public double calculateShootSpeed() {
        double distanceMeters = getHubDistance();
        if (distanceMeters < 0)
            return 20.0;

        double distanceInches = distanceMeters * 39.3701;
        if (distanceInches < 120)
            return 15.0;
        if (distanceInches > 220)
            return 28.0;
        return 0.05625 * (distanceInches - 120) + 15.5;
    }

    public PhotonCamera getFrontLeftCamera() {
        return frontLeftCamera;
    }

    public PhotonCamera getFrontRightCamera() {
        return frontRightCamera;
    }

    public void simulationPeriodic(Pose2d robotSimPose) {
        if (visionSim != null)
            visionSim.update(robotSimPose);
    }

    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation() && visionSim != null)
            visionSim.resetRobotPose(pose);
    }
}