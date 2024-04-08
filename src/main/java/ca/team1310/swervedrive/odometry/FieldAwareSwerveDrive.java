package ca.team1310.swervedrive.odometry;

import ca.team1310.swervedrive.core.CoreSwerveDrive;
import ca.team1310.swervedrive.core.config.CoreSwerveConfig;
import ca.team1310.swervedrive.odometry.hardware.MXPNavX;
import ca.team1310.swervedrive.telemetry.FieldAwareDriveTelemetry;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class FieldAwareSwerveDrive extends CoreSwerveDrive {
    private final Gyro                     gyro;
    private final Field2d                  field;
    private final SwerveDrivePoseEstimator estimator;

    public FieldAwareSwerveDrive(CoreSwerveConfig cfg) {
        super(cfg);
        this.gyro      = new MXPNavX();
        this.field     = new Field2d();
        this.estimator = new SwerveDrivePoseEstimator(
            kinematics,
            gyro.getRotation2d(),
            getModulePositions(),
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    }

    protected void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> deviation) {
        estimator.addVisionMeasurement(pose, timestampSeconds, deviation);
    }

    public void updateOdometry() {
        estimator.update(gyro.getRotation2d(), getModulePositions());
        field.setRobotPose(estimator.getEstimatedPosition());
    }

    public void resetOdometry(Pose2d pose) {
        estimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public Rotation3d getGyroRotation3d() {
        return new Rotation3d(gyro.getRoll(), gyro.getPitch(), gyro.getYaw());
    }

    public void zeroGyro() {
        gyro.zeroGyro();
    }

    public final FieldAwareDriveTelemetry getFieldTelemetryState() {
        return new FieldAwareDriveTelemetry(
            gyro.getTelemetryState(),
            field,
            getPose());
    }
}
