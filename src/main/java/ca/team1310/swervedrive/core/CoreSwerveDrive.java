package ca.team1310.swervedrive.core;

import ca.team1310.swervedrive.core.config.CoreSwerveConfig;
import ca.team1310.swervedrive.core.config.ModuleConfig;
import ca.team1310.swervedrive.telemetry.CoreSwerveDriveTelemetry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.Arrays;

public class CoreSwerveDrive {
    private final SwerveModule[]          modules;
    protected final SwerveDriveKinematics kinematics;
    private final double                  robotPeriodSeconds;
    private final double                  maxModuleMPS;
    private final double                  maxTranslationMPS;
    private final double                  maxOmegaRadPerSec;
    private final double                  trackWidthMetres;
    private final double                  wheelBaseMetres;
    private final double                  wheelRadiusMetres;

    private ChassisSpeeds                 desiredChassisSpeeds;

    public CoreSwerveDrive(CoreSwerveConfig cfg) {
        // order matters in case we want to use AdvantageScope
        modules                 = new SwerveModule[4];
        modules[0]              = createModule(cfg.frontLeftModuleConfig());
        modules[1]              = createModule(cfg.frontRightModuleConfig());
        modules[2]              = createModule(cfg.backLeftModuleConfig());
        modules[3]              = createModule(cfg.backRightModuleConfig());

        kinematics              = new SwerveDriveKinematics(
            Arrays.stream(modules).map(SwerveModule::getLocation).toArray(Translation2d[]::new));

        this.robotPeriodSeconds = cfg.robotPeriodSeconds();
        this.maxModuleMPS       = cfg.maxAttainableModuleSpeedMetresPerSecond();
        this.maxTranslationMPS  = cfg.maxAttainableTranslationSpeedMetresPerSecond();
        this.maxOmegaRadPerSec  = cfg.maxAchievableRotationalVelocityRadiansPerSecond();

        this.trackWidthMetres   = cfg.trackWidthMetres();
        this.wheelBaseMetres    = cfg.wheelBaseMetres();
        this.wheelRadiusMetres  = cfg.wheelRadiusMetres();
    }

    private SwerveModule createModule(ModuleConfig cfg) {
        if (RobotBase.isSimulation()) {
            return new SwerveModuleSimulation(cfg);
        }
        return new SwerveModuleImpl(cfg);
    }

    protected final SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    protected Pose2d[] getModulePoses(Pose2d robotPose) {
        return Arrays.stream(modules).map(m -> {
            Transform2d tx = new Transform2d(m.getLocation(), m.getState().angle);
            return robotPose.plus(tx);
        }).toArray(Pose2d[]::new);
    }

    protected SwerveModuleState[] getStates() {
        return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    public void setModuleState(String moduleName, SwerveModuleState desiredState) {
        // This is for TEST MODE ONLY!!! Not for internal use or drive use.
        for (SwerveModule module : modules) {
            if (module.getName().equals(moduleName)) {
                module.setDesiredState(desiredState);
                break;
            }
        }
    }


    public final void drive(ChassisSpeeds rawDesiredRobotOrientedVelocity) {
        this.desiredChassisSpeeds = rawDesiredRobotOrientedVelocity;
        updateModules();
    }

    private void updateModules() {
        // calculate desired states
        Translation2d       centerOfRotation = new Translation2d();
        SwerveModuleState[] states           = kinematics.toSwerveModuleStates(desiredChassisSpeeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, desiredChassisSpeeds,
            maxModuleMPS, maxTranslationMPS, maxOmegaRadPerSec);

        // set states
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    public final boolean lock() {

        // safety code to prevent locking if robot is moving
        for (SwerveModule swerveModule : modules) {
            // do not lock if moving more than 4cm/s
            if (Math.abs(swerveModule.getState().speedMetersPerSecond) > 0.04) {
                return false;
            }
        }

        desiredChassisSpeeds = new ChassisSpeeds(0, 0, 0);

        // set speed to 0 and angle wheels to center
        for (SwerveModule module : modules) {
            module.setDesiredState(new SwerveModuleState(0.0, module.getPosition().angle));
        }

        // tell kinematics that we aren't moving
        kinematics.toSwerveModuleStates(new ChassisSpeeds());
        return true;
    }

    public CoreSwerveDriveTelemetry getCoreTelemetry() {
        ChassisSpeeds measuredChassisSpeeds = kinematics.toChassisSpeeds(getStates());

        return new CoreSwerveDriveTelemetry(
            this.maxModuleMPS,
            this.maxTranslationMPS,
            this.maxOmegaRadPerSec,
            this.trackWidthMetres,
            this.wheelBaseMetres,
            this.wheelRadiusMetres,
            modules[0].getModuleTelemetry(),
            modules[1].getModuleTelemetry(),
            modules[2].getModuleTelemetry(),
            modules[3].getModuleTelemetry(),
            desiredChassisSpeeds,
            measuredChassisSpeeds);
    }
}
