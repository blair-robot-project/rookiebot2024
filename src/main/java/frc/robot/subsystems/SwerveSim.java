package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

import static edu.wpi.first.math.kinematics.SwerveModuleState.optimize;
import static frc.robot.allConstants.driveConstants.*;
import static frc.robot.allConstants.driveConstants.turnPIDkd;

public class SwerveSim extends SwerveModule{
    private DutyCycleEncoderSim simturnEncoder = new DutyCycleEncoderSim(turnEncoder);
    private EncoderSim simdriveEncoder = new EncoderSim((Encoder) driveEncoder); // figure this out later, how to sim a relative encoder
    private AnalogGyro gyro = new AnalogGyro(1); // 1 is a filler value, not yet sure what to put into the Analog Gyro
    private AnalogGyroSim simGyro = new AnalogGyroSim(gyro);
    public SwerveSim(int driveMotor, int turnMotor) {
        super(driveMotor, turnMotor);
        drivePid = new PIDController(drivePIDkp, drivePIDki, drivePIDkd);
        turnPid=new PIDController(turnPIDkp,turnPIDki,turnPIDkd);
        driveEncoder = this.driveMotor.getEncoder(); // need this to be a sim encoder

    }
    public void SetDesired(SwerveModuleState desiredState) {

    }
    public void simulationPeriodic() {
    }

}
