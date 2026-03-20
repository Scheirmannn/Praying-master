package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

    // ── Constants ─────────────────────────────────────────────────

    private static final String CAMERA_NAME = "LeftShooterCam";

    // comp is 24 in
    private static final double CORRECTION_OFFSET = 0.0;

    private static final double CENTER_TO_BUMPER_INCHES = 11.5;
    private static final double CAMERA_OFFSET_INCHES = 8.0;

    // ── Hardware ──────────────────────────────────────────────────

    private final PhotonCamera m_camera;

    public VisionSubsystem() {
        m_camera = new PhotonCamera(CAMERA_NAME);
        SmartDashboard.putNumber("Distance to Target (in)", 0);
        SmartDashboard.putBoolean("Target Visible", false);
        SmartDashboard.putNumber("Target Yaw", 0);
        SmartDashboard.putNumber("Alignment Yaw", 0);
        SmartDashboard.putNumber("Calculated Shoot Speed", 0);
        HttpCamera m_stream = new HttpCamera(CAMERA_NAME, "http://10.97.90.11:1182/stream.mjpg");
        CameraServer.addCamera(m_stream);
    }

    // ── Periodic ─────────────────────────────────────────────────

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Target Visible", hasTarget());
        SmartDashboard.putNumber("Distance to Target (in)", getDistanceToTarget());
        SmartDashboard.putNumber("Target Yaw", getTargetYaw());
        SmartDashboard.putNumber("Alignment Yaw", getAlignmentYaw());
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

        return (distanceMeters * 39.3701) + CENTER_TO_BUMPER_INCHES + CORRECTION_OFFSET;
    }

    // Yaw adjusted for camera offset - use for shooting
    public double getTargetYaw() {
        var target = getValidTarget();
        if (target == null) return 0;

        double rawYaw = target.getYaw();
        double distanceInches = getDistanceToTarget();
        if (distanceInches < 0) return rawYaw;

        double offsetCorrectionDeg = Math.toDegrees(
            Math.atan2(CAMERA_OFFSET_INCHES, distanceInches)
        );

        return rawYaw - offsetCorrectionDeg;
    }

    // Raw yaw with no offset correction - use for alignment
    public double getAlignmentYaw() {
        var target = getValidTarget();
        if (target == null) return 0;
        return target.getYaw();
    }

    public boolean isAligned() {
        return hasTarget() && Math.abs(getAlignmentYaw()) < 2.0;
    }

    public double calculateShootSpeed() {
        double distanceInches = getDistanceToTarget();
        if (distanceInches < 0) return 20.0;
        if (distanceInches < 120) return 15.0;
        if (distanceInches > 220) return 28.0;
        return 0.05625 * (distanceInches - 120) + 15.5;
    }
}