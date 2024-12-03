// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.claw.clawConstants.*;


public class ClawSubsystem extends SubsystemBase {
    CANSparkMax motor;
    CANSparkMax inverseFollowMotor;

    /**
     * Creates a new Claw Subsystem.
     */
    public ClawSubsystem() {

        motor = new CANSparkMax(clawConstants.CLAW_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        inverseFollowMotor = new CANSparkMax(clawConstants.CLAW_FOLLOWER_ID, CANSparkLowLevel.MotorType.kBrushless);
        inverseFollowMotor.follow(motor, true);
    }


    /**
     * Intake command factory method.
     *
     * @return a command
     */
    public Command Intake() {
        return runOnce(
                () -> {
                    this.motor.setVoltage(CLAW_INTAKE_VOLTAGE);
                });
    }

    /**
     * Outtake command factory method.
     *
     * @return a command
     */
    public Command Outtake() {
        return runOnce(() -> {
            this.motor.setVoltage(CLAW_OUTTAKE_VOLTAGE);
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
            this.motor.setVoltage(CLAW_HOLD_VOLTAGE);

        });
    }



    @Override
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Claw Sim Voltage");
        builder.addDoubleProperty("Voltage",() -> motor.getBusVoltage(),null);
    }



    public void periodic() {
        // This method will be called once per scheduler run
    }


    @Override
    public void simulationPeriodic() {
        DCMotorSim motorSim = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
        motorSim.setInputVoltage(motor.getBusVoltage());





        // This method will be called once per scheduler run during simulation
    }
}
