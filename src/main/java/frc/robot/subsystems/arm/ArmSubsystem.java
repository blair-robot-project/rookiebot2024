// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArmSubsystem extends SubsystemBase {

    //arm motor
    CANSparkMax armMotor;
    //second arm motor
    CANSparkMax armMotorFollower;
    //establishing kp, ki, and kd
    double kP = armConstants.armKP, kI = armConstants.armKI, kD = armConstants.armKD;
    //pid controller
    PIDController pid = new PIDController(kP, kI, kD);
    /*
    * current is the arm's current position in radians
    * */
    double currentState = armConstants.armBasePosition;
    double desired = armConstants.armHighScorePosition;

    //arm sim stuff

    CANSparkMax simMotor = new CANSparkMax(armConstants.armSimID, CANSparkLowLevel.MotorType.kBrushless);

    private DutyCycleEncoder armEncoder = new DutyCycleEncoder(armConstants.encoderPort);

    private double armSetpointDegrees = armConstants.armDesiredValue;

    private final PIDController armPIDController = new PIDController(kP, kI, kD);

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    private final Mechanism2d mech2d = new Mechanism2d(60, 60);

    private final MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);

    private final MechanismLigament2d armTower =
            armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            armConstants.armGearbox,
            armConstants.armGearing,
            armConstants.armInertia,
            armConstants.armLength,
            armConstants.minAngleRads,
            armConstants.maxAngleRads,
            armConstants.armSimGrav,
            armConstants.armBasePosition
    );

    private final MechanismLigament2d armLigament =
            armPivot.append(
                    new MechanismLigament2d(
                            "Arm",
                            30,
                            Units.radiansToDegrees(armSim.getAngleRads()),
                            6,
                            new Color8Bit(Color.kYellow)
                    )
            );

    //feed forward
    SimpleMotorFeedforward feedForward_a = new SimpleMotorFeedforward(armConstants.armFeedForwardKs, armConstants.armFeedForwardKv, armConstants.armFeedForwardKa);

    public ArmSubsystem() {
        armMotor = new CANSparkMax(armConstants.armMotorIDa, MotorType.kBrushless);
        armMotorFollower= new CANSparkMax(armConstants.armMotorFollowerID, MotorType.kBrushless);
        armMotorFollower.follow(armMotor, false);

        armEncoder.setDistancePerRotation(armConstants.kArmEncoderDistPerPulse);

        armTower.setColor(new Color8Bit(Color.kBlue));

        // Set the Arm position setpoint and P constant to Preferences if the keys don't already exist
        Preferences.initDouble(armConstants.kArmPositionKey, armConstants.armDesiredValue);
        Preferences.initDouble(armConstants.kArmPKey, armConstants.armKP);
    }

    public double getArmF(double des){
        return feedForward_a.calculate(des);
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command setVoltage(double voltage) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    armMotor.setVoltage(voltage);
                });
    }

    public Command stopRunning() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    armMotor.setVoltage(0);
                });
    }

    public Command goToStow() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    desired = armConstants.armStowPosition;
                });
    }

    public Command goToHighScore() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    desired = armConstants.armHighScorePosition;
                });
    }

    public Command goToHalf() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    desired = armConstants.armHighScorePosition / 2;
                });
    }

    public Command goToIntake() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    desired = armConstants.armBasePosition;
                });
    }

    public double returnMotorPos() {
        return armEncoder.getDistance();
    }

    /** Load setpoint and kP from preferences. */
    public void loadPreferences() {
        // Read Preferences for Arm setpoint and kP on entering Teleop
        armSetpointDegrees = Preferences.getDouble(armConstants.kArmPositionKey, armSetpointDegrees);
        if (kP != Preferences.getDouble(armConstants.kArmPKey, kP)) {
            kP = Preferences.getDouble(armConstants.kArmPKey, kP);
            armPIDController.setP(kP);
        }
    }


    public void reachSetpoint() {
        var pidOutput =
                armPIDController.calculate(
                        returnMotorPos(), Units.degreesToRadians(armSetpointDegrees));
        simMotor.setVoltage(pidOutput);
    }

    public void stop() {
        simMotor.set(0.0);
    }

    public void close() {
        simMotor.close();
        armEncoder.close();
        mech2d.close();
        armPivot.close();
        armPIDController.close();
        armLigament.close();
    }

    public BooleanSupplier isDone(){
        BooleanSupplier finished = () ->
        returnMotorPos()/armConstants.armGearRatio==this.desired;
        return finished;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Arm Sim Voltage");
        builder.publishConstString("1.0", "Logging stuff");builder.addDoubleProperty("1.1 position", this::returnMotorPos, null);
    }

    @Override
    public void periodic() {
        currentState = returnMotorPos() / armConstants.armGearRatio; // gear ratio maybe somewhere?
        double voltage = pid.calculate(currentState, desired) + getArmF(desired);
        setVoltage(voltage);
        armLigament.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

        SmartDashboard.putData("arm sim", mech2d);
    }

    @Override
    /** Run the control loop to reach and maintain the setpoint from the preferences. */
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        armSim.setInput(simMotor.get() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        armSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        currentState = armSim.getAngleRads();
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

        // Update the Mechanism Arm angle based on the simulated arm angle
        armLigament.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    }

}