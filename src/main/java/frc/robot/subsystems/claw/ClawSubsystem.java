// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static frc.robot.subsystems.claw.clawConstants.*;


public class ClawSubsystem extends SubsystemBase {
    CANSparkMax motor;
    RelativeEncoder encoder;
    double voltage = 0;
    /**
     * Creates a new Claw Subsystem.
     */
    public ClawSubsystem() {

        motor = new CANSparkMax(clawConstants.CLAW_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();

        this.motor.restoreFactoryDefaults();
        this.motor.setSmartCurrentLimit(60);
        this.motor.setIdleMode(CANSparkBase.IdleMode.kCoast);

        this.motor.burnFlash();
    }



    /**
     * Sets motor to intake voltage form clawConstants
     *
     * @return command that sets the claw motor voltage to intake value
     */
    public Command Intake() {
        return runOnce(
                () -> {
                    voltage = CLAW_INTAKE_VOLTAGE;
                    motor.setVoltage(voltage);
                });
    }

    /**
     * Sets motor to outtake voltage from clawConstants
     *
     * @return command that sets the claw motor voltage to outtake value
     */
    public Command Outtake() {
        return runOnce(() -> {
            voltage = CLAW_OUTTAKE_VOLTAGE;
            motor.setVoltage(voltage);
        });
    }

    /**
     * DoNothing command factory method.
     *
     * @return command that stops the claw motor
     */
   public Command doNothing() {

        return runOnce(() -> {
            this.motor.stopMotor();
        });
    }


    /**
     * HoldBucket command factory method.
     * (Currently not used as the robot does not need
     * the motors to be moving to hold a buck)
     *
     * @return command that tensions the motor to hold a bucket
     */
    public Command HoldBucket() {

        return runOnce(() -> {
            voltage = CLAW_HOLD_VOLTAGE;
            motor.setVoltage(voltage);

        });
    }

    /**
     * Used to get voltage in motor for logging
     *
     * @return the voltage currently going into the claw motor
     */
    public double getVoltage() {
        if (Robot.isSimulation()) {
            return motor.getAppliedOutput() * RobotController.getBatteryVoltage();
        } else {
            return voltage;
        }
    }

    /**
     * Used to get encoder position for logging
     *
     * @return the encoder position
     */
    public double getDistance() {
        return encoder.getPosition();
    }

    /**
     * Sets smart dashboard values for logging
     *
     * @param builder sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Claw Sim Voltage");
        builder.addDoubleProperty("Voltage", this::getVoltage,null);
        builder.addDoubleProperty("Distance", this::getDistance, null);
    }



    public void periodic() {
        // This method will be called once per scheduler run
    }


    /**
     * Sets voltage during simulation
     *
     */
    @Override
    public void simulationPeriodic() {
        DCMotorSim motorSim = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
        motorSim.setInputVoltage(motor.getBusVoltage());
    }
}
