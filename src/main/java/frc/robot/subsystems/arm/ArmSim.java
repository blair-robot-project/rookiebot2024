package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
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

public class ArmSim implements AutoCloseable {
    CANSparkMax simMotor = new CANSparkMax(armConstants.armSimID, CANSparkLowLevel.MotorType.kBrushless);
    Encoder armEncoder = new Encoder(armConstants.encoderAChannel, armConstants.encoderBChannel);
    /*
    gearbox - The type of and number of motors in the arm gearbox.
    gearing - The gearing of the arm (numbers greater than 1 represent reductions).
    jKgMetersSquared - The moment of inertia of the arm, can be calculated from CAD software.
    armLengthMeters - The length of the arm.
    minAngleRads - The minimum angle that the arm is capable of.
    maxAngleRads - The maximum angle that the arm is capable of.
    simulateGravity - Whether gravity should be simulated or not.
    startingAngleRads - The initial position of the Arm simulation in radians.
    */
    //smth from reo
    // The P gain for the PID controller that drives this arm.
    private double armKP = armConstants.armKP;
    private double armSetpointDegrees = armConstants.armDesiredValue;
    private final PIDController armPIDController = new PIDController(armKP, 0, 0);
    private final Encoder encoder =
            new Encoder(armConstants.encoderAChannel, armConstants.encoderBChannel);

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            armConstants.armGearbox,
            armConstants.armGearing,
            armConstants.armInertia,
            armConstants.armLength,
            armConstants.minAngleRads,
            armConstants.maxAngleRads,
            armConstants.armSimGrav,
            armConstants.armBaseValue
    );
    // The P gain for the PID controller that drives this arm.

    // Simulation classes help us simulate what's going on, including gravity.
    // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
    // to 255 degrees (rotated down in the back).
    private final EncoderSim encoderSim = new EncoderSim(armEncoder);

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    private final Mechanism2d mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d armTower =
            armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
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

    /** Creates a new ExampleSubsystem. */
    public ArmSim() {
        armEncoder.setDistancePerPulse(armConstants.kArmEncoderDistPerPulse);

        // Put Mechanism 2d to SmartDashboard
        SmartDashboard.putData("Arm Sim", mech2d);
        armTower.setColor(new Color8Bit(Color.kBlue));

        // Set the Arm position setpoint and P constant to Preferences if the keys don't already exist
        Preferences.initDouble(armConstants.kArmPositionKey, armConstants.armDesiredValue);
        Preferences.initDouble(armConstants.kArmPKey, armConstants.armKP);
    }

    public void simulationPeriodic() {
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        armSim.setInput(simMotor.get() * RobotController.getBatteryVoltage());

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

    /** Load setpoint and kP from preferences. */
    public void loadPreferences() {
        // Read Preferences for Arm setpoint and kP on entering Teleop
        armSetpointDegrees = Preferences.getDouble(armConstants.kArmPositionKey, armSetpointDegrees);
        if (armKP != Preferences.getDouble(armConstants.kArmPKey, armKP)) {
            armKP = Preferences.getDouble(armConstants.kArmPKey, armKP);
            armPIDController.setP(armKP);
        }
    }

    /** Run the control loop to reach and maintain the setpoint from the preferences. */
    public void reachSetpoint() {
        var pidOutput =
            armPIDController.calculate(
                encoder.getDistance(), Units.degreesToRadians(armSetpointDegrees));
        simMotor.setVoltage(pidOutput);
    }

    public void stop() {
        simMotor.set(0.0);
    }

    @Override
    public void close() {
        simMotor.close();
        encoder.close();
        mech2d.close();
        armPivot.close();
        armPIDController.close();
        armLigament.close();
    }


}
