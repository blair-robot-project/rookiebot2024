package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.allConstants.driveConstants;

import static edu.wpi.first.math.kinematics.SwerveModuleState.optimize;
import static frc.robot.allConstants.driveConstants.*;

// the encoders commands don't work bc we're using a diff type of encoder
// idk how to get distance traveled on a relative encoder

public class SwerveModule {
    /**
     * private static final double WHEEL_RADIUS = 0.0508;
     * //private static final double WHEEL_CIRCUMFERENCE=2*WHEEL_RADIUS*Math.PI;
     * //private static final int kEncoderResolution = 4096;
     * //private static final double kModuleMaxAngularVelocity = swervedrive.MAX_ANGULAR_SPEED;
     * //private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
     **/
    CANSparkMax driveMotor;
    CANSparkMax turnMotor;
    double driveVoltage;
    double turnVoltage;
    RelativeEncoder driveEncoder;
    PIDController drivePid;
    PIDController turnPid;
    DutyCycleEncoder turnEncoder;


    // ks = volts
    // kv = volts * seconds / distance
    // ka = volts * seconds^2 / distance
    private final SimpleMotorFeedforward feedForward_d = new SimpleMotorFeedforward(swerveFeedForwardDriveKs, swerveFeedForwardDriveKv, swerveFeedForwardDriveKa);
    private final SimpleMotorFeedforward feedForward_t = new SimpleMotorFeedforward(swerveFeedForwardTurnKs, swerveFeedForwardTurnKv, swerveFeedForwardTurnKa);

    public SwerveModule(int driveMotor, int turnMotor, int turnEncoder) {
        this.driveMotor = new CANSparkMax(driveMotor, CANSparkLowLevel.MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotor, CANSparkLowLevel.MotorType.kBrushless);
        drivePid = new PIDController(drivePIDkp, drivePIDki, drivePIDkd);
        turnPid = new PIDController(turnPIDkp, turnPIDki, turnPIDkd);
        driveEncoder = this.driveMotor.getEncoder();
        this.turnEncoder = new DutyCycleEncoder(turnEncoder);


    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition() * WHEEL_CIRCUMFERENCE * driveGearing, new Rotation2d(turnEncoder.getAbsolutePosition() / (2 * Math.PI)));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveEncoder.getPosition() * WHEEL_CIRCUMFERENCE * driveGearing, new Rotation2d(turnEncoder.getAbsolutePosition() / (2 * Math.PI)));
    }

    public void SetDesired(SwerveModuleState desiredState) {
        turnEncoder.setDistancePerRotation(WHEEL_CIRCUMFERENCE);

        var encoderRotation = new Rotation2d(turnEncoder.getDistance());
        optimize(desiredState, encoderRotation);

        final double driveOutput = drivePid.calculate(driveEncoder.getVelocity(), desiredState.speedMetersPerSecond);
        final double drive_feedforward = feedForward_d.calculate(desiredState.speedMetersPerSecond);

        final double turnOutput = turnPid.calculate(turnEncoder.getAbsolutePosition() / (2 * Math.PI), desiredState.angle.getRadians());

        driveMotor.setVoltage(driveOutput + drive_feedforward);
        turnMotor.setVoltage(turnOutput + feedForward_t.ks);
    }


}
