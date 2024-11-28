package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

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
    //double driveVoltage;
    //double turnVoltage;
    RelativeEncoder driveEncoder;
    PIDController drivePid;
    PIDController turnPid;
    DutyCycleEncoder turnEncoder;
    boolean turnEncoderInverted;


    // ks = volts
    // kv = volts * seconds / distance
    // ka = volts * seconds^2 / distance
    private final SimpleMotorFeedforward feedForward_d = new SimpleMotorFeedforward(swerveFeedForwardDriveKs, swerveFeedForwardDriveKv, swerveFeedForwardDriveKa);
    private final SimpleMotorFeedforward feedForward_t = new SimpleMotorFeedforward(swerveFeedForwardTurnKs, swerveFeedForwardTurnKv, swerveFeedForwardTurnKa);

    public SwerveModule(int driveMotor, int turnMotor, int turnEncoder, boolean driveMotorInverted, boolean turnMotorInverted, boolean turnEncoderInverted) {
        this.driveMotor = new CANSparkMax(driveMotor, CANSparkLowLevel.MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotor, CANSparkLowLevel.MotorType.kBrushless);
        drivePid = new PIDController(drivePIDkp, drivePIDki, drivePIDkd);
        turnPid = new PIDController(turnPIDkp, turnPIDki, turnPIDkd);
        turnPid.enableContinuousInput(0,2*Math.PI);
        driveEncoder = this.driveMotor.getEncoder();
        this.turnEncoder = new DutyCycleEncoder(turnEncoder);
        //this.turnEncoder.setDistancePerRotation();
        this.driveEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE/driveGearing);
        this.driveEncoder.setVelocityConversionFactor((WHEEL_CIRCUMFERENCE/driveGearing)/60);
        this.driveMotor.setInverted(driveMotorInverted);
        this.turnMotor.setInverted(turnMotorInverted);
        this.turnEncoderInverted=turnEncoderInverted;
        this.turnMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        this.driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        this.turnMotor.burnFlash();
        this.driveMotor.burnFlash();
    }

    public double wheelPointing() {
        return (driveEncoder.getPosition() - Math.floor(driveEncoder.getPosition())*2*Math.PI);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(), new Rotation2d((turnEncoder.getAbsolutePosition() - turnEncoder.getPositionOffset()) / (2 * Math.PI)));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveEncoder.getVelocity(), new Rotation2d((turnEncoder.getAbsolutePosition()-turnEncoder.getPositionOffset()) / (2 * Math.PI)));
    }

    public void SetDesired(SwerveModuleState desiredState) {
        var encoderRotation = Rotation2d.fromRotations(MathUtil.inputModulus(turnEncoder.getAbsolutePosition(), -0.5, 0.5));

        //var encoderRotation = new Rotation2d(turnEncoder.getDistance());
        optimize(desiredState, encoderRotation);

        final double driveOutput = drivePid.calculate(driveEncoder.getVelocity(), desiredState.speedMetersPerSecond);
        final double drive_feedforward = feedForward_d.calculate(desiredState.speedMetersPerSecond);

        final double turnOutput = turnPid.calculate((encoderRotation.getRadians()-turnEncoder.getPositionOffset()/(2*Math.PI)), desiredState.angle.getRadians());

        driveMotor.setVoltage(driveOutput + drive_feedforward);
        turnMotor.setVoltage(turnOutput + feedForward_t.ks);
    }

    public void setVoltage(double voltage, CANSparkMax motor){
        motor.setVoltage(voltage);
    }
}
