package frc.robot.subsystems.swerve;

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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;

import static edu.wpi.first.math.kinematics.SwerveModuleState.optimize;
import static frc.robot.subsystems.swerve.driveConstants.*;

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
    double turnOffset;
    Rotation2d encoderRotation;
    SwerveModuleState desiredState;


    // ks = volts
    // kv = volts * seconds / distance
    // ka = volts * seconds^2 / distance
    private final SimpleMotorFeedforward feedForward_d = new SimpleMotorFeedforward(swerveFeedForwardDriveKs, swerveFeedForwardDriveKv, swerveFeedForwardDriveKa);
    private final SimpleMotorFeedforward feedForward_t = new SimpleMotorFeedforward(swerveFeedForwardTurnKs, swerveFeedForwardTurnKv, swerveFeedForwardTurnKa);

    public SwerveModule(
            int driveMot, int turnMot, int turnEnc,
            boolean driveMotInv, boolean turnMotInv, boolean turnEncInv,
            double turnOff
    ) {
        driveMotor = new CANSparkMax(driveMot, CANSparkLowLevel.MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMot, CANSparkLowLevel.MotorType.kBrushless);
        drivePid = new PIDController(drivePIDkp, drivePIDki, drivePIDkd);
        turnPid = new PIDController(turnPIDkp, turnPIDki, turnPIDkd);
        turnPid.enableContinuousInput(-Math.PI,Math.PI);
        driveEncoder = this.driveMotor.getEncoder();
        turnEncoder = new DutyCycleEncoder(turnEnc);
        //this.turnEncoder.setDistancePerRotation();
        driveEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE/driveGearing);
        driveEncoder.setVelocityConversionFactor((WHEEL_CIRCUMFERENCE/driveGearing)/60);
        driveMotor.setInverted(driveMotInv);
        turnMotor.setInverted(turnMotInv);
        turnEncoderInverted=turnEncInv;
        //this.turnEncoder.setPositionOffset(turnOffset);
        turnOffset = turnOff;
        turnMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT);
        turnMotor.setSmartCurrentLimit(TURN_CURRENT_LIMIT);
        turnMotor.burnFlash();
        driveMotor.burnFlash();

        desiredState = new SwerveModuleState();

    }

    public double wheelPointing() {
        return (driveEncoder.getPosition() - Math.floor(driveEncoder.getPosition())*2*Math.PI);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(), new Rotation2d(getTurnPosition()));
    }

    public double getTurnPosition(){
       return MathUtil.angleModulus((turnEncoder.getAbsolutePosition())*2*Math.PI - this.turnOffset);
    }


    public double getVelocity(){
        return getState().speedMetersPerSecond;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveEncoder.getVelocity(), new Rotation2d(getTurnPosition()));
    }

    public double getDesiredSpeed(){
        return desiredState.speedMetersPerSecond;
    }
    public double getDesiredAngle(){
        return desiredState.angle.getRadians();
    }
    public void SetDesired(SwerveModuleState desiredState) {
        this.desiredState=desiredState;
        if (this.turnEncoderInverted) {
            encoderRotation = Rotation2d.fromRotations(MathUtil.inputModulus(1-(turnEncoder.getAbsolutePosition()-turnOffset), 0.0, 1.0));
        }
        else {
            encoderRotation = Rotation2d.fromRotations(MathUtil.inputModulus(turnEncoder.getAbsolutePosition()-turnOffset, 0.0, 1.0));
        }

        desiredState = optimize(desiredState, encoderRotation);

        final double driveOutput = drivePid.calculate(driveEncoder.getVelocity(), desiredState.speedMetersPerSecond);
        final double drive_feedforward = feedForward_d.calculate(desiredState.speedMetersPerSecond);

        final double turnOutput = turnPid.calculate(encoderRotation.getRadians(), desiredState.angle.getRadians());

        driveMotor.setVoltage(driveOutput + drive_feedforward);
        turnMotor.setVoltage(turnOutput + feedForward_t.ks);
    }

    public void setVoltage(double voltage){
        driveMotor.setVoltage(voltage);
    }


public double getVoltage(){
    return driveMotor.get() * RobotController.getBatteryVoltage();
}


}

