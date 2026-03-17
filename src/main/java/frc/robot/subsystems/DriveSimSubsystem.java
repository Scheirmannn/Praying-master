package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSimSubsystem extends SubsystemBase {

    private final DriveSubsystem m_drive;
    private final Field2d field = new Field2d();
    private Pose2d simPose = new Pose2d();

    public DriveSimSubsystem(DriveSubsystem drive) {
        m_drive = drive;
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        double dt = 0.02;
        double heading = DriveSubsystem.getSimGyroAngleRad();

        simPose = new Pose2d(
            simPose.getX() + DriveSubsystem.kSimVxMPS * dt,
            simPose.getY() + DriveSubsystem.kSimVyMPS * dt,
            new Rotation2d(heading)
        );

        field.setRobotPose(simPose);
        m_drive.getVisionSubsystem().simulationPeriodic(simPose);
    }

    public void resetSimPose(Pose2d pose) {
        simPose = pose;
        DriveSubsystem.setSimGyroAngleRad(pose.getRotation().getRadians());
        m_drive.resetOdometry(pose);
    }
}