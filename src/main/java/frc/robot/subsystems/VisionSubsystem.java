package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionSubsystem extends SubsystemBase {

    // ── Constants ─────────────────────────────────────────────────

    private static final String CAMERA_NAME = "LeftShooterCam";

    // TODO: Replace with actual camera position relative to robot center
    // x = forward/back in meters (positive = forward)
    // y = left/right in meters (positive = left)
    // z = height in meters
    private static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
        new Translation3d(11.5* 0.0254, 8.0 * 0.0254, 20.5 * 0.0254),
        new Rotation3d(0, Math.toRadians(-15), 0)
    );

    // TODO: Replace with actual robot half length in inches
    private static final double CENTER_TO_BUMPER_INCHES = 11.5;

    // Field length
    private static final double FIELD_LENGTH_METERS = 651 * 0.0254; // 16.535m

    // Red target - 158in from right, 182in up
    private static final double RED_TARGET_X = 12.522;
    private static final double RED_TARGET_Y = 4.623;

    // Blue target - mirrored
    private static final double BLUE_TARGET_X = 4.013;
    private static final double BLUE_TARGET_Y = 4.623;

    // ── Hardware ──────────────────────────────────────────────────

    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator m_photonEstimator;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Field2d m_field = new Field2d();

    public VisionSubsystem(SwerveDrivePoseEstimator poseEstimator) {
        m_poseEstimator = poseEstimator;

        m_camera = new PhotonCamera(CAMERA_NAME);

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        m_photonEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            CAMERA_TO_ROBOT
        );

        SmartDashboard.putData("Vision Field", m_field);
        SmartDashboard.putNumber("Distance to Target (in)", 0);
        SmartDashboard.putBoolean("Target Visible", false);
    }

    // ── Periodic ─────────────────────────────────────────────────

    @Override
    public void periodic() {
        var result = m_camera.getLatestResult();

        SmartDashboard.putBoolean("Target Visible", result.hasTargets());

        if (result.hasTargets()) {
            m_photonEstimator.update(result).ifPresent(estimatedPose -> {
                m_poseEstimator.addVisionMeasurement(
                    estimatedPose.estimatedPose.toPose2d(),
                    estimatedPose.timestampSeconds
                );
            });
        }

        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
        SmartDashboard.putNumber("Distance to Target (in)", getDistanceToTarget());
    }

    // ── Public methods ────────────────────────────────────────────

    public Pose2d getEstimatedPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public double getDistanceToTarget() {
        Pose2d pose = m_poseEstimator.getEstimatedPosition();

        double targetX;
        double targetY;

        if (DriverStation.getAlliance().isPresent() &&
            DriverStation.getAlliance().get() == Alliance.Blue) {
            targetX = BLUE_TARGET_X;
            targetY = BLUE_TARGET_Y;
        } else {
            targetX = RED_TARGET_X;
            targetY = RED_TARGET_Y;
        }

        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        double distanceInches = Math.sqrt(dx * dx + dy * dy) * 39.3701;
        return distanceInches + CENTER_TO_BUMPER_INCHES;
    }

    public boolean hasTarget() {
        return m_camera.getLatestResult().hasTargets();
    }

    public double idealShooterSpeed() {
        double distance = getDistanceToTarget();
        if (distance < 120.0) return 15.0;
        if (distance > 240.0) return 28.0;
        
        return 0.05625 *(distance - 120.0) + 15.5;
    }
}