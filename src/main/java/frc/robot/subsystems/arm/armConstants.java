package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class armConstants {
    //motor id UNDETERMINED
    public static final int armMotorIDa = 62;
    //motor id for sim
    public static final int armSimID=19;
    //motor follower id UNDETERMINED
    public static final int armMotorFollowerID = 2;
    //motor kp, ki, kd UNDETERMINED
    public static final double armKP = 0.1, armKI = 0, armKD = 0;
    //default desired arm value RADIANS UNDETERMINED
    public static final double armHighScorePosition = 0.25;
    //default base arm value RADIANS UNDETERMINED
    public static final double armBasePosition = 0;
    //arm angle at the top RADIANS UNDETERMINED
    public static final double armStowPosition = 0.5;
    //gear ratio UNDETERMINED
    public static final double armGearRatio = 1;
    //desired arm value UNDETERMINED RADIANS
    public static final double armDesiredValue = 0.25;

    //
    public static final double armFeedForwardKs=1;
    public static final double armFeedForwardKv=1;
    public static final double armFeedForwardKa=1;

    //VALUES NEEDED FOR ARM SIM
    //ALL VALUES BELOW ARE FAKE AND TEMPORARY

    //arm dcmotor for arm sim FINALIZED I THINK?
    public static DCMotor armGearbox = DCMotor.getNEO(1);
    //gearing UNFINALIZED
    public static double armGearing = 1;
    //moment inertia of the arm NEED FROM CAD (jkg per meters squared)
    //UNFINALIZED
    public static double armInertia = 200;
    //arm length (units?) UNFINALIZED
    public static double armLength = Units.inchesToMeters(30);
    //minAngle (radians) UNFINALIZED
    public static double minAngleRads = Units.degreesToRadians(-60);
    //maxAngle (rads) UNFINALIZED
    public static double maxAngleRads = Units.degreesToRadians(120);
    //whether or not to simualte gravity FINALIZED
    public static boolean armSimGrav = true;

    //Encoder Channels UNFINALIZED
    public static int encoderAChannel = 9;
    public static int encoderBChannel = 8;

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    //  = (2 * PI rads) / (4096 pulses)
    // UNFINALIZED
    public static double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
    public static final String kArmPositionKey = "ArmPosition";
    public static final String kArmPKey = "ArmP";

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
}
