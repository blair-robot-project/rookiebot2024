package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.math.kinematics.SwerveModuleState.optimize;

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
    DutyCycleEncoder turn_encoder;

    PIDController pid1;

    public swervemodule(
            int drivemotor,
            int turnmotor,
            int turn_encoder1
    )
    {
        drivemotor1 = new CANSparkMax(drivemotor, CANSparkLowLevel.MotorType.kBrushless);
        turnmotor1 = new CANSparkMax(turnmotor, CANSparkLowLevel.MotorType.kBrushless);
        pid1 = new PIDController(0.25,0.0,0.0);
        drive_encoder =drivemotor1.getEncoder();
        turn_encoder=new DutyCycleEncoder(turn_encoder1);
    }

    public void moving() {
        turnmotor1.setVoltage(turnvoltage1);
        drivemotor1.setVoltage(drivevoltage1);
    }

    // return the current position of the module
    // idk how you would do this
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                drive_encoder.getCountsPerRevolution(), new Rotation2d(turn_encoder.getDistance()));
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(drive_encoder.getCountsPerRevolution(),new Rotation2d(turn_encoder.getDistance()));
    }

    public void set(SwerveModuleState desiredState){
        var encoderRotation=new Rotation2d(turn_encoder.getDistance());

        // optimization things that don't work
        desiredState.optimize(encoderRotation);
        desiredState.cosineScale(encoderRotation);
        final double driveOutput = pid1.calculate(drive_encoder.getRate(),desiredState.speedMetersPerSecond);
        // feed forward thing
    }


    public void periodic(){
        turn_encoder.setDistancePerRotation(360);
        double a= drive_encoder.getPosition();
        double b= turn_encoder.getAbsolutePosition();

        System.out.println(a);
        System.out.println(b);
    }
    public void SetDesired(SwerveModuleState desiredState) {
        var encoderRotation = new Rotation2d(turn_encoder.getDistance());
        optimize(desiredState, encoderRotation);
    }

}
