package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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
    RelativeEncoder turnEncoder;
    PIDController pid;



    // ks = volts
    // kv = volts * seconds / distance
    // ka = volts * seconds^2 / distance
    private final SimpleMotorFeedforward feedForward_d = new SimpleMotorFeedforward(1,2,3);
    private final SimpleMotorFeedforward feedForward_t = new SimpleMotorFeedforward(1,2,3);

    public SwerveModule(int driveMotor, int turnMotor) {
        this.driveMotor = new CANSparkMax(driveMotor, CANSparkLowLevel.MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotor, CANSparkLowLevel.MotorType.kBrushless);
        pid = new PIDController(swervePIDkp,swervePIDki,swervePIDkd);
        driveEncoder = this.driveMotor.getEncoder();
        //turn_encoder=new DutyCycleEncoder(turn_encoder1);
        turnEncoder = this.turnMotor.getEncoder();
    }

    public void moving() {
        turnMotor.setVoltage(turnVoltage);
        driveMotor.setVoltage(driveVoltage);
    }

    // return the current position of the module
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getCountsPerRevolution(), new Rotation2d(turnEncoder.getPosition()));
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getCountsPerRevolution(),new Rotation2d(turnEncoder.getPosition()));
    }

    public void SetDesired(SwerveModuleState desiredState) {
        var encoderRotation = new Rotation2d(turnEncoder.getPosition());
        optimize(desiredState, encoderRotation);

        final double driveOutput = pid.calculate(driveEncoder.getVelocity()*kWheelCircumference/60, desiredState.speedMetersPerSecond);
        final double drive_feedforward = feedForward_d.calculate(desiredState.speedMetersPerSecond);
        final double turnOutput= pid.calculate(turnEncoder.getPosition(),desiredState.angle.getRadians());

        driveMotor.setVoltage(driveOutput+drive_feedforward);
        turnMotor.setVoltage(turnOutput+feedForward_t.ks);
    }

}
