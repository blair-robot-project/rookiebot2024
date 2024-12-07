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
        this.motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        this.motor.burnFlash();
    }



    /**
     * Intake command factory method.
     *
     * @return a command
     */
    public Command Intake() {
        return runOnce(
                () -> {
                    voltage = CLAW_INTAKE_VOLTAGE;
                    motor.setVoltage(voltage);
                });
    }

    /**
     * Outtake command factory method.
     *
     * @return a command
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
     * @return a command
     */
   public Command doNothing() {

        return runOnce(() -> {
            this.motor.stopMotor();
        });
    }


    /**
     * HoldBucket command factory method.
     *
     * @return a command
     */
    public Command HoldBucket() {

        return runOnce(() -> {
            voltage = CLAW_HOLD_VOLTAGE;
            motor.setVoltage(voltage);

        });
    }


    public double getVoltage() {
        if (Robot.isSimulation()) {
            return motor.getAppliedOutput() * RobotController.getBatteryVoltage();
        } else {
            return voltage;
        }
    }

    public double getDistance() {
        return encoder.getPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Claw Sim Voltage");
        builder.addDoubleProperty("Voltage", this::getVoltage,null);
        builder.addDoubleProperty("Distance", this::getDistance, null);
    }



    public void periodic() {
        // This method will be called once per scheduler run
    }


    @Override
    public void simulationPeriodic() {
        DCMotorSim motorSim = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
        motorSim.setInputVoltage(motor.getBusVoltage());
    }
}
