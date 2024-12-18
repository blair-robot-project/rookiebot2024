package frc.robot.subsystems.swerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.lang.reflect.Field;

import static edu.wpi.first.math.kinematics.SwerveModuleState.optimize;
import static frc.robot.subsystems.swerve.SwerveModule.*;
import static frc.robot.subsystems.swerve.driveConstants.*;

public class SwerveSim extends SwerveModule {
    private SimpleMotorFeedforward simfeedForward_d = new SimpleMotorFeedforward(swerveFeedForwardDriveKs, swerveFeedForwardDriveKv, swerveFeedForwardDriveKa);
    private SimpleMotorFeedforward simfeedForward_t = new SimpleMotorFeedforward(swerveFeedForwardTurnKs, swerveFeedForwardTurnKv, swerveFeedForwardTurnKa);
    private DutyCycleEncoderSim simturnEncoder = new DutyCycleEncoderSim(turnEncoder);
    private EncoderSim simdriveEncoder = new EncoderSim((Encoder) driveEncoder);
    private double simcurrentPos = 0;
    private boolean FieldOriented;
    private double gyroSim;
    SwerveDriveKinematics kinematics;
    Field2d field2d;
    double trackWidth;
    double wheelBase;
    double xShift;
    double maxRotSpeed;
    double maxLinearSpeed;







    // private AnalogGyro gyro = new AnalogGyro(1); // 1 is a filler value, not yet sure what to put into the Analog Gyro
    // private AnalogGyroSim simGyro = new AnalogGyroSim(gyro);
    public SwerveSim(int driveMotor, int turnMotor, int turnEncoder, boolean driveMotorInverted, boolean turnMotorInverted, boolean turnEncoderInverted, double turnOffset) {
        super(driveMotor, turnMotor, turnEncoder, driveMotorInverted, turnMotorInverted, turnEncoderInverted, turnOffset);

        drivePid = new PIDController(drivePIDkp, drivePIDki, drivePIDkd);
        turnPid = new PIDController(turnPIDkp, turnPIDki, turnPIDkd);
        double lastTime= Timer.getFPGATimestamp();
        Pose2d odoPose= new Pose2d();
        Rotation2d currHeading=new Rotation2d();

/*
        SwerveDriveOdometry odometry= new SwerveDriveOdometry(
                kinematics,
                currHeading,
                positions(),
        );


 */



    }



    public void snapToDesired(SwerveModuleState desiredState) {
        simdriveEncoder.setDistancePerPulse(WHEEL_CIRCUMFERENCE);

        var encoderRotation = new Rotation2d(simdriveEncoder.getDistance());
        optimize(desiredState, encoderRotation);

        final double driveOutput = drivePid.calculate(simdriveEncoder.getRate() * 60, desiredState.speedMetersPerSecond);
        final double drive_feedforward = simfeedForward_d.calculate(desiredState.speedMetersPerSecond);

        final double turnOutput = turnPid.calculate(simturnEncoder.getAbsolutePosition() / (2 * Math.PI), desiredState.angle.getRadians());

        driveMotor.setVoltage(driveOutput + drive_feedforward);
        turnMotor.setVoltage(turnOutput + simfeedForward_t.ks);
    }

    public SwerveModuleState getsimState() {
        return new SwerveModuleState(
                simdriveEncoder.getDistance() * WHEEL_CIRCUMFERENCE * driveGearing, new Rotation2d(simturnEncoder.getAbsolutePosition() / (2 * Math.PI))); // Apparently the position needs to be in radians
    }


    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Swerve Sim");



    }

    public void simulationPeriodic() {


    }

}
