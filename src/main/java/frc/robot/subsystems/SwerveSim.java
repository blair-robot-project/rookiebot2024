package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.allConstants.driveConstants;

import static edu.wpi.first.math.kinematics.SwerveModuleState.optimize;
import static frc.robot.allConstants.driveConstants.*;

public class SwerveSim extends SwerveModule {
    private final SimpleMotorFeedforward simfeedForward_d = new SimpleMotorFeedforward(swerveFeedForwardDriveKs, swerveFeedForwardDriveKv, swerveFeedForwardDriveKa);
    private final SimpleMotorFeedforward simfeedForward_t = new SimpleMotorFeedforward(swerveFeedForwardTurnKs, swerveFeedForwardTurnKv, swerveFeedForwardTurnKa);
    private DutyCycleEncoderSim simturnEncoder = new DutyCycleEncoderSim(turnEncoder);
    private EncoderSim simdriveEncoder = new EncoderSim((Encoder) driveEncoder); // figure this out later, how to sim a relative encoder

    // private AnalogGyro gyro = new AnalogGyro(1); // 1 is a filler value, not yet sure what to put into the Analog Gyro
    // private AnalogGyroSim simGyro = new AnalogGyroSim(gyro);
    public SwerveSim(int driveMotor, int turnMotor, int turnEncoder) {
        super(driveMotor, turnMotor, turnEncoder);
        drivePid = new PIDController(drivePIDkp, drivePIDki, drivePIDkd);
        turnPid = new PIDController(turnPIDkp, turnPIDki, turnPIDkd);
        driveEncoder = this.driveMotor.getEncoder(); // need this to be a sim encoder

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

    public void simulationPeriodic() {

    }

}
