// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
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
import frc.robot.Robot;

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

    //current is the arm's current position in radians
    double currentState = armConstants.armBasePosition;

    //desired is where the robot wants to go
    double desired = armConstants.armStowPosition;
    String desiredName = "Stow";

    double voltage = 0.0;

    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(armConstants.encoderPort);

    private final PIDController armPIDController = new PIDController(kP, kI, kD);

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    private Mechanism2d mech2d;

    private MechanismRoot2d armPivot;

    private EncoderSim encoderSim;

    private MechanismLigament2d armTower;

    private SingleJointedArmSim armSim;

    private MechanismLigament2d armLigament;

    //feed forward
    ArmFeedforward feedForward_a = new ArmFeedforward(armConstants.armFeedForwardKs, armConstants.armFeedForwardKg, armConstants.armFeedForwardKv);

    public ArmSubsystem() {
        armMotor = new CANSparkMax(armConstants.armMotorIDa, MotorType.kBrushless);
        armMotorFollower = new CANSparkMax(armConstants.armMotorFollowerID, MotorType.kBrushless);
        armMotorFollower.follow(armMotor, false);
        armEncoder.reset();
        armEncoder.setDistancePerRotation(armConstants.kArmEncoderDistPerRotation);

        if (Robot.isSimulation()) {

            //constructing arm sim stuff
            armSim = new SingleJointedArmSim(
                    armConstants.armGearbox,
                    armConstants.armGearRatio,
                    armConstants.armInertia,
                    armConstants.armLength,
                    armConstants.minAngleRads,
                    armConstants.maxAngleRads,
                    armConstants.armSimGrav,
                    armConstants.armBasePosition
            );

            mech2d = new Mechanism2d(60, 60);

            armPivot = mech2d.getRoot("ArmPivot", 30, 30);

            armTower = armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));

            encoderSim = new EncoderSim(new Encoder(armConstants.kEncoderAChannel, armConstants.kEncoderBChannel));
            encoderSim.setDistancePerPulse(armConstants.kArmEncoderDistPerPulse);

            armLigament =
                    armPivot.append(
                            new MechanismLigament2d(
                                    "Arm",
                                    30,
                                    Units.radiansToDegrees(armSim.getAngleRads()),
                                    6,
                                    new Color8Bit(Color.kYellow)
                            )
                    );

            SmartDashboard.putData("Arm Sim", mech2d);
            armTower.setColor(new Color8Bit(Color.kBlue));

            // Set the Arm position setpoint and P constant to Preferences if the keys don't already exist
            Preferences.initDouble(armConstants.kArmPositionKey, desired);
            Preferences.initDouble(armConstants.kArmPKey, kP);
        }
    }

    //getters
    public double getVoltage() { return voltage; }
    public double getSetpoint() { return desired; }
    public String getSetpointName() { return desiredName; }
    public double getCurrentState() { return armEncoder.getPositionOffset() / armConstants.armGearRatio; }
    public double getSimState() {
        if(encoderSim == null) {
            return 0.0;
        } else {
            return encoderSim.getDistance() / armConstants.armGearRatio;
        }
    }
    public double getArmF (boolean sim, double des) {
        if(sim) {
            return feedForward_a.calculate(getCurrentState(), des);
        } else {
            return feedForward_a.calculate(getSimState(), des);
        }
    }
    /**
     * Example command factory method.
     *
     * @return a command
     */
    public void setVoltage(double voltage) {
        armMotor.setVoltage(voltage);
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
                    desiredName = "Stow";
                });
    }

    public Command goToHighScore() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    desired = armConstants.armHighScorePosition;
                    desiredName = "High Score";
                });
    }

    public Command goToHalf() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    desired = armConstants.armHighScorePosition / 2;
                    desiredName = "Half";
                });
    }

    public Command goToIntake() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    desired = armConstants.armBasePosition;
                    desiredName = "Intake";
                });
    }

    /** Load setpoint and kP from preferences. */
    public void loadPreferences() {
        // Read Preferences for Arm setpoint and kP on entering Teleop
        desired = Preferences.getDouble(armConstants.kArmPositionKey, desired);
        if (kP != Preferences.getDouble(armConstants.kArmPKey, kP)) {
            kP = Preferences.getDouble(armConstants.kArmPKey, kP);
            armPIDController.setP(kP);
        }
    }

    public void stop() {
        armMotor.set(0.0);
    }

    public void close() {
        armMotor.close();
        armEncoder.close();
        mech2d.close();
        armPivot.close();
        armPIDController.close();
        armLigament.close();
    }

    public BooleanSupplier isDone(){
        BooleanSupplier finished = () ->
        getCurrentState() == desired;
        return finished;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Arm Sim Voltage");
        builder.publishConstString("1.0", "Logging stuff");
        builder.addDoubleProperty("1.1 position", this::getCurrentState, null);
        builder.addDoubleProperty("1.2 voltage", this::getVoltage, null);
        builder.addDoubleProperty( "1.3 setpoint", this::getSetpoint, null);
        builder.addDoubleProperty("1.4 sim position", this::getSimState, null);
        builder.addStringProperty("1.5 setpoint name", this::getSetpointName, null);
    }

    @Override
    public void periodic() {
        currentState = getCurrentState();
        voltage = pid.calculate(currentState, desired) + getArmF(false, desired);
        setVoltage(voltage);
    }

    @Override
    /** Run the control loop to reach and maintain the setpoint from the preferences. */
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        armMotor.setVoltage(
                armPIDController.calculate(
                        encoderSim.getDistance(), Units.degreesToRadians(desired)) + getArmF(true, desired));
        voltage = armMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        armSim.setInputVoltage(voltage);

        // Next, we update it. The standard loop time is 20ms.
        armSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        encoderSim.setDistance(armSim.getAngleRads());

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

        // Update the Mechanism Arm angle based on the simulated arm angle
        armLigament.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    }

    public void print(Object o) {
        System.out.println(o.toString());
    }

}