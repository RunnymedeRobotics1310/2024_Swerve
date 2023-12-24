package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants.SwerveModule;

public class SwerveDriveSubsystem extends SubsystemBase {

    private final CANSparkMax frontLeftDriveMotor   = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax frontLeftTurnMotor    = new CANSparkMax(11, MotorType.kBrushless);
    private final CANCoder    frontLeftAngleSensor  = new CANCoder(12);

    private final CANSparkMax frontRightDriveMotor  = new CANSparkMax(20, MotorType.kBrushless);
    private final CANSparkMax frontRightTurnMotor   = new CANSparkMax(21, MotorType.kBrushless);
    private final CANCoder    frontRightAngleSensor = new CANCoder(22);

    private final CANSparkMax backRightDriveMotor   = new CANSparkMax(30, MotorType.kBrushless);
    private final CANSparkMax backRightTurnMotor    = new CANSparkMax(31, MotorType.kBrushless);
    private final CANCoder    backRightAngleSensor  = new CANCoder(32);

    private final CANSparkMax backLeftDriveMotor    = new CANSparkMax(35, MotorType.kBrushless);
    private final CANSparkMax backLeftTurnMotor     = new CANSparkMax(36, MotorType.kBrushless);
    private final CANCoder    backLeftAngleSensor   = new CANCoder(37);

    public void setMotorSpeeds(SwerveModule swerveModule, double speed, double turn) {

        CANSparkMax driveMotor, turnMotor;

        switch (swerveModule) {

        case FRONT_LEFT:
            driveMotor = frontLeftDriveMotor;
            turnMotor = frontLeftTurnMotor;
            break;

        case FRONT_RIGHT:
            driveMotor = frontRightDriveMotor;
            turnMotor = frontRightTurnMotor;
            break;

        case BACK_LEFT:
            driveMotor = backLeftDriveMotor;
            turnMotor = backLeftTurnMotor;
            break;

        case BACK_RIGHT:
            driveMotor = backRightDriveMotor;
            turnMotor = backRightTurnMotor;
            break;

        default:
            return;
        }

        setMotorSpeeds(driveMotor, turnMotor, speed, turn);
    }

    private void setMotorSpeeds(CANSparkMax driveMotor, CANSparkMax turnMotor, double speed, double turn) {
        driveMotor.set(speed);
        turnMotor.set(turn);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Front Left Drive Speed", frontLeftDriveMotor.get());
        SmartDashboard.putNumber("Front Left Turn Speed", frontLeftTurnMotor.get());
        SmartDashboard.putNumber("Front Left Angle", frontLeftAngleSensor.getPosition());

        SmartDashboard.putNumber("Front Right Drive Speed", frontRightDriveMotor.get());
        SmartDashboard.putNumber("Front Right Turn Speed", frontRightTurnMotor.get());
        SmartDashboard.putNumber("Front Right Angle", frontRightAngleSensor.getPosition());

        SmartDashboard.putNumber("Back Left Drive Speed", backLeftDriveMotor.get());
        SmartDashboard.putNumber("Back Left Turn Speed", backLeftTurnMotor.get());
        SmartDashboard.putNumber("Back Left Angle", backLeftAngleSensor.getPosition());

        SmartDashboard.putNumber("Back Right Drive Speed", backRightDriveMotor.get());
        SmartDashboard.putNumber("Back Right Turn Speed", backRightTurnMotor.get());
        SmartDashboard.putNumber("Back Right Angle", backRightAngleSensor.getPosition());
    }

}

