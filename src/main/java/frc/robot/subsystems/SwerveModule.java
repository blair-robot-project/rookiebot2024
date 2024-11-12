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

import static edu.wpi.first.math.kinematics.SwerveModuleState.optimize;
import static frc.robot.allConstants.driveConstants.*;

// the encoders commands don't work bc we're using a diff type of encoder
// idk how to get distance traveled on a relative encoder

public class SwerveModule {
    /**private static final double kWheelRadius = 0.0508;
    //private static final double kWheelCircumference=2*kWheelRadius*Math.PI;
    //private static final int kEncoderResolution = 4096;
    //private static final double kModuleMaxAngularVelocity = swervedrive.kMaxAngularSpeed;
    //private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
    **/
    CANSparkMax driveMotor;
    CANSparkMax turnMotor;
    double driveVoltage;
    double turnVoltage;
    RelativeEncoder driveEncoder;
    DutyCycleEncoder turnEncoder;
    PIDController drivePid;
    PIDController turnPid;



    // ks = volts
    // kv = volts * seconds / distance
    // ka = volts * seconds^2 / distance
    private final SimpleMotorFeedforward feedForward_d = new SimpleMotorFeedforward(swerveFeedForwardDriveKs, swerveFeedForwardDriveKv, swerveFeedForwardDriveKa);
    private final SimpleMotorFeedforward feedForward_t = new SimpleMotorFeedforward(swerveFeedForwardTurnKs, swerveFeedForwardTurnKv, swerveFeedForwardTurnKa);

    public SwerveModule(int driveMotor, int turnMotor) {
        this.driveMotor = new CANSparkMax(driveMotor, CANSparkLowLevel.MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotor, CANSparkLowLevel.MotorType.kBrushless);
        drivePid = new PIDController(drivePIDkp, drivePIDki, drivePIDkd);
        turnPid=new PIDController(turnPIDkp,turnPIDki,turnPIDkd);
        driveEncoder = this.driveMotor.getEncoder();
    }


    public void moving() {
        turnMotor.setVoltage(turnVoltage);
        driveMotor.setVoltage(driveVoltage);
    }

    // return the current position of the module
    // wrong, expects distance in meters and a rotation2d representing the angle of the wheel
    // the rotation2d being passed in is wrong, it expects radians by default
    // - james p
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition()*kWheelCircumference*driveGearing, new Rotation2d(turnEncoder.getAbsolutePosition()/(2*Math.PI)));
    }

    // never used, idk if we have to
    // same problem as in getposition
    public SwerveModuleState getState(){
        return new SwerveModuleState(
                driveEncoder.getPosition()*kWheelCircumference*driveGearing,new Rotation2d(turnEncoder.getAbsolutePosition()/(2*Math.PI)));
    }

    public void SetDesired(SwerveModuleState desiredState) {
        var encoderRotation = new Rotation2d(turnEncoder.getDistance());
        optimize(desiredState, encoderRotation);

        final double driveOutput = drivePid.calculate(driveEncoder.getVelocity()*kWheelCircumference/60, desiredState.speedMetersPerSecond);
        final double drive_feedforward = feedForward_d.calculate(desiredState.speedMetersPerSecond);
        final double turnOutput= turnPid.calculate(turnEncoder.getDistance(),desiredState.angle.getRadians());

        driveMotor.setVoltage(driveOutput+drive_feedforward);
        turnMotor.setVoltage(turnOutput+feedForward_t.ks);
    }



}
