package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class armConstants {
    //motor id UNDETERMINED
    public static final int armMotorIDa = 62;
    //motor follower id UNDETERMINED
    public static final int armMotorFollowerID = 2;
    //motor kp, ki, kd UNDETERMINED
    public static final double armKP = 6.6, armKI = 0.6, armKD = 0.01;
    //arm angle at the top RADIANS
    public static final double armStowPosition = -0.05;
    //default desired arm value RADIANS
    public static final double armHighScorePosition = 0.14;
    //default half position

    public static final double armHalfPosition = 0.2;
    //default base arm value RADIANS
    public static final double armIntakePosition = 0.45;

    //gear ratio
    public static final double armGearRatio = 2.0/3;

    //feed forward values
    public static final double armFeedForwardKs=0.5;
    public static final double armFeedForwardKg=0.55;
    public static final double armFeedForwardKv=0.0;

    public static final double offset = 0.091588 + 0.032357;

    //encoder sim values
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;


    //VALUES NEEDED FOR ARM SIM

    //arm dcmotor for arm sim FINALIZED I THINK?
    public static DCMotor armGearbox = DCMotor.getNEO(1);
    //moment inertia of the arm NEED FROM CAD (jkg per meters squared) UNFINALIZED
    public static double armInertia = 7;
    //arm length (units?) UNFINALIZED
    public static double armLength = Units.inchesToMeters(30);
    //minAngle (radians) UNFINALIZED
    public static double minAngleRads = Units.degreesToRadians(-30);
    //maxAngle (rads) UNFINALIZED
    public static double maxAngleRads = Units.degreesToRadians(120);
    //whether or not to simualte gravity FINALIZED
    public static boolean armSimGrav = true;

    public static boolean armInversion = true;

    //Encoder Channels UNFINALIZED
    public static int encoderPort = 1;

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    //  = (2 * PI rads) / (4096 pulses)
    // UNFINALIZED
    public static double kArmEncoderDistPerPulse = 2 * Math.PI / 4096;
    public static int kArmEncoderDistPerRotation = 1;
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
