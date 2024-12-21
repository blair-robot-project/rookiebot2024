// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.claw.clawConstants.*;


public class ClawSubsystem extends SubsystemBase {
    CANSparkMax motor;
    boolean hasOutput;
    String state;
    /**
     * Creates a new Claw Subsystem.
     */
    public ClawSubsystem() {
        motor = new CANSparkMax(clawConstants.CLAW_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        hasOutput = false;
        state = "doing nothing";
    }



    /**
     * Intake command factory method.
     *
     * @return a command
     */
    public Command intake() {
        return runOnce(
                () -> {
                    motor.setVoltage(CLAW_INTAKE_VOLTAGE);
                    hasOutput = false;
                    state = "intaking";
                });
    }

    /**
     * Outtake command factory method.
     *
     * @return a command
     */
    public Command outtake() {
        return runOnce(() -> {
            motor.setVoltage(CLAW_OUTTAKE_VOLTAGE);
            hasOutput = true;
            state = "outtaking";
        });
    }

    /**
     * DoNothing command factory method.
     *
     * @return a command
     */
   public Command doNothing() {

        return runOnce(() -> {
            motor.stopMotor();
            state = "doing nothing";
        });
    }


    /**
     * HoldBucket command factory method.
     *
     * @return a command
     */
    public Command holdBucket() {

        return runOnce(() -> {
            if(hasOutput) {
                motor.setVoltage(CLAW_HOLD_VOLTAGE);
                state = "holding";
            } else {
                doNothing();
            }
        });
    }


    public double getVoltage() { return motor.getAppliedOutput() * RobotController.getBatteryVoltage(); }
    public String getState() { return state; }
    public boolean getHasOutputted() { return hasOutput; }
    @Override
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Claw Sim Voltage");
        builder.addDoubleProperty("Voltage", this::getVoltage,null);
        builder.addStringProperty("State", this::getState, null);
        builder.addBooleanProperty("hasOutputted", this::getHasOutputted, null);
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
