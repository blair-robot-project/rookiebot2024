package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class swervemodule extends SubsystemBase {
    CANSparkMax drivemotor1;
    CANSparkMax turnmotor1;
    double voltage11;
    double voltage21;
    double drivevoltage1;
    double turnvoltage1;
    DutyCycleEncoder driveread1;
    DutyCycleEncoder turnread1;

    PIDController pid1;

    public swervemodule(
            int driveMotorChannel,
            int turningMotorChannel,
            int driveEncoderChannelA,
            int driveEncoderChannelB,
            int turningEncoderChannelA,
            int turningEncoderChannelB
    )
    {
        drivemotor1 = new CANSparkMax(driveMotorChannel, CANSparkLowLevel.MotorType.kBrushless);
        turnmotor1 = new CANSparkMax(turningMotorChannel, CANSparkLowLevel.MotorType.kBrushless);
        pid1 = new PIDController(0.25,0.0,0.0);
        driveread1 =new DutyCycleEncoder(driveEncoderChannelA);
        turnread1=new DutyCycleEncoder(turningEncoderChannelA);
    }

    public void moving() {
        turnmotor1.setVoltage(turnvoltage1);
        drivemotor1.setVoltage(drivevoltage1);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveread1.getDistance(), new Rotation2d(turnread1.getDistance()));
    }

    public void periodic(){
        turnread1.setDistancePerRotation(360);
        driveread1.setDistancePerRotation(360);
        double a= driveread1.getAbsolutePosition();
        double b= turnread1.getAbsolutePosition();

        System.out.println(a);
        System.out.println(b);
    }

}
