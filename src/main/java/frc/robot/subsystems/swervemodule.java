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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.math.kinematics.SwerveModuleState.optimize;
import static edu.wpi.first.units.Units.*;


// the encoders commands don't work bc we're using a diff type of encoder
// idk how to get distance traveled on a relative encoder


public class swervemodule extends SubsystemBase {
    private static final double kWheelRadius = 0.0508;
    private static final double kWheelCircumference=2*kWheelRadius*Math.PI;
    private static final int kEncoderResolution = 4096;
    private static final double kModuleMaxAngularVelocity = swervedrive.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
    CANSparkMax drivemotor1;
    CANSparkMax turnmotor1;
    double voltage1;
    double voltage2;
    double drivevoltage1;
    double turnvoltage1;
    RelativeEncoder drive_encoder;
    RelativeEncoder turn_encoder;

    PIDController pid1;

    // ks = volts
    // kv = volts * seconds / distance
    // ka = volts * seconds^2 / distance
    private final SimpleMotorFeedforward feedforward_d = new SimpleMotorFeedforward(1,2,3);
    private final SimpleMotorFeedforward feedforward_t = new SimpleMotorFeedforward(1,2,3);

    public swervemodule(
            int drivemotor,
            int turnmotor
    )
    {
        drivemotor1 = new CANSparkMax(drivemotor, CANSparkLowLevel.MotorType.kBrushless);
        turnmotor1 = new CANSparkMax(turnmotor, CANSparkLowLevel.MotorType.kBrushless);
        pid1 = new PIDController(0.25,0.0,0.0);
        drive_encoder =drivemotor1.getEncoder();
        //turn_encoder=new DutyCycleEncoder(turn_encoder1);
        turn_encoder=turnmotor1.getEncoder();
    }

    public void moving() {
        turnmotor1.setVoltage(turnvoltage1);
        drivemotor1.setVoltage(drivevoltage1);
    }

    // return the current position of the module
    // idk how you would do this
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                drive_encoder.getCountsPerRevolution(), new Rotation2d(turn_encoder.getPosition()));
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(drive_encoder.getCountsPerRevolution(),new Rotation2d(turn_encoder.getPosition()));
    }

    public void set(SwerveModuleState desiredState){
        var encoderRotation=new Rotation2d(turn_encoder.getPosition());

        // optimization things that don't work
        optimize(desiredState, encoderRotation);
        //desiredState.cosineScale(encoderRotation);

        // kWheelCircumference

        final double driveOutput = pid1.calculate(drive_encoder.getVelocity()*kWheelCircumference/60, desiredState.speedMetersPerSecond);
        final double drive_feedforward = feedforward_d.calculate(desiredState.speedMetersPerSecond);
        final double turnOutput=pid1.calculate(turn_encoder.getPosition(),desiredState.angle.getRadians());
        final double turn_feedforward = feedforward_t.calculate(RadiansPerSecond.of(turn_encoder.getVelocity())).in(Volts);


        drivemotor1.setVoltage(driveOutput+drive_feedforward);
        turnmotor1.setVoltage(turnOutput+turn_feedforward);
    }

    public void periodic(){
        turn_encoder.setDistancePerRotation(2*Math.PI);
        double a = drive_encoder.getPosition();
        double b = turn_encoder.getPosition();

        System.out.println(a);
        System.out.println(b);
    }
    public void SetDesired(SwerveModuleState desiredState) {
        var encoderRotation = new Rotation2d(turn_encoder.getPosition());
        optimize(desiredState, encoderRotation);
    }

}
