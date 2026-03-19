package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

    // ── Constants ─────────────────────────────────────────────────

    // TODO: Replace with actual camera name configured in PhotonVision UI
    private static final String CAMERA_NAME = "YOUR_CAMERA_NAME";

    // TODO: Replace with actual camera position relative to robot center
    // x = forward/back in meters (positive = forward)
    // y = left/right in meters (positive = left)
    // z = height in meters
    private static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
        new Translation3d(0.0, 0.0, 0.5),
        new Rotation3d(0, Math.toRadians(-15), 0)
    );

    // TODO: Replace with actual robot half length in inches
    private static final double CENTER_TO_BUMPER_INCHES = 14.0;

    // TODO: measure how far left or right camera is from robot center in inches
    // positive = camera is to the left of center
    private static final double CAMERA_OFFSET_INCHES = 8.0;

    // ── Hardware ──────────────────────────────────────────────────

    private final PhotonCamera m_camera;

    public VisionSubsystem() {
        m_camera = new PhotonCamera(CAMERA_NAME);
        SmartDashboard.putNumber("Distance to Target (in)", 0);
        SmartDashboard.putBoolean("Target Visible", false);
        SmartDashboard.putNumber("Target Yaw", 0);
        SmartDashboard.putNumber("Calculated Shoot Speed", 0);
    }

    // ── Periodic ─────────────────────────────────────────────────

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Target Visible", hasTarget());
        SmartDashboard.putNumber("Distance to Target (in)", getDistanceToTarget());
        SmartDashboard.putNumber("Target Yaw", getTargetYaw());
        SmartDashboard.putNumber("Calculated Shoot Speed", calculateShootSpeed());
    }

    // ── Target filtering ─────────────────────────────────────────

    private PhotonTrackedTarget getValidTarget() {
        var result = m_camera.getLatestResult();
        if (!result.hasTargets()) return null;

        int[] validTagIds;
        if (DriverStation.getAlliance().isPresent() &&
            DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            validTagIds = new int[]{25, 26};
        } else {
            validTagIds = new int[]{9, 10};
        }

        for (var target : result.getTargets()) {
            for (int id : validTagIds) {
                if (target.getFiducialId() == id) {
                    return target;
                }
            }
        }
        return null;
    }

    // ── Public methods ────────────────────────────────────────────

    public boolean hasTarget() {
        return getValidTarget() != null;
    }

    public double getDistanceToTarget() {
        var target = getValidTarget();
        if (target == null) return -1;

        double distanceMeters = target.getBestCameraToTarget()
            .getTranslation()
            .getNorm();

        return (distanceMeters * 39.3701) + CENTER_TO_BUMPER_INCHES;
    }

    // Yaw to target adjusted for camera offset
    // Returns degrees - negative = target is to the left, positive = to the right
    public double getTargetYaw() {
        var target = getValidTarget();
        if (target == null) return 0;

        // raw yaw from camera
        double rawYaw = target.getYaw();

        // adjust for camera not being centered
        // camera offset creates an angular offset we need to correct for
        double distanceInches = getDistanceToTarget();
        if (distanceInches < 0) return rawYaw;

        double offsetCorrectionDeg = Math.toDegrees(
            Math.atan2(CAMERA_OFFSET_INCHES, distanceInches)
        );

        return rawYaw - offsetCorrectionDeg;
    }

    public boolean isAligned() {
        return Math.abs(getTargetYaw()) < 2.0; // within 2 degrees
    }

    public double calculateShootSpeed() {
        double distanceInches = getDistanceToTarget();
        if (distanceInches < 0) return 20.0;
        if (distanceInches < 120) return 15.0;
        if (distanceInches > 220) return 28.0;
        return 0.05625 * (distanceInches - 120) + 15.5;
    }
}