/**package frc.robot.allConstants.driveConstants

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swervedrive;

object driveConstants{
  const val WHEEL_RADIUS = 0.0508;
  const val WHEEL_CIRCUMFERENCE=2*WHEEL_RADIUS*Math.PI;
  const val kEncoderResolution = 4096;
  const val kModuleMaxAngularVelocity = swervedrive.MAX_ANGULAR_SPEED;
  const val kModuleMaxAngularAcceleration = 2 * Math.PI;

// the locations of the wheels
  const val m_frontLeftLocation = (0.381, 0.381);
  const val m_frontRightLocation = (0.381, -0.381)
  const val m_backLeftLocation = (-0.381, 0.381);
  const val m_backRightLocation = (-0.381, -0.381);
}
**/

package frc.robot.allConstants;

public final class driveConstants {
        public static final double MAX_SPEED =3.0; // placeholder, 3 m/s
        public static final double MAX_ANGULAR_SPEED =Math.PI; // placeholder, 1/2 rotation per second (in radians, so pi radians is 1/2 of a rotation)

        public static final double WHEEL_RADIUS = 0.0508; // placeholder
        public static final double WHEEL_CIRCUMFERENCE =2* WHEEL_RADIUS *Math.PI; // placeholder
        /*
        public static final double kEncoderResolution = 4096;
        public static final double kModuleMaxAngularVelocity = MAX_ANGULAR_SPEED;
        public static final double kModuleMaxAngularAcceleration = 2 * Math.PI;
         */
        // idk if we'll need these ^^^
        //location of the wheels relative to the center of the robot
        public static final double moduleDistanceX = 0.381; // placeholder
        public static final double moduleDistanceY = 0.381; // placeholder

        //PID calculator values for swerve

        public static final double drivePIDkp = 0.25; // placeholder
        public static final double drivePIDki = 0.0; // placeholder
        public static final double drivePIDkd = 0.0; // placeholder

        public static final double turnPIDkp = 0.25; // placeholder
        public static final double turnPIDki = 0.0; // placeholder
        public static final double turnPIDkd = 0.0; // placeholder

        //feedForward calc values for swerve
        public static final double swerveFeedForwardDriveKs = 1; // placeholder
        public static final double swerveFeedForwardDriveKv = 1; // placeholder
        public static final double swerveFeedForwardDriveKa = 1; // placeholder

        public static final double swerveFeedForwardTurnKs = 1; // placeholder
        public static final double swerveFeedForwardTurnKv = 1; // placeholder
        public static final double swerveFeedForwardTurnKa = 1; // placeholder

        //8 motor ids
        public static final int driveMotor1 = 20; // placeholder
        public static final int turnMotor1 = 21; // placeholder

        public static final int driveMotor2 = 17; // placeholder
        public static final int turnMotor2 = 61; // placeholder

        public static final int driveMotor3 = 1; // placeholder
        public static final int turnMotor3 = 2; // placeholder

        public static final int driveMotor4 = 4; // placeholder
        public static final int turnMotor4 = 3; // placeholder

        //the interavals in which the motor loops
        public static final double pdsec = 0.02; // always the same I think

        //no matter the robot position forward is forward (fRel = field relative)
        public static final boolean fRel = true; // probably will stay this way


        public static final double driveGearing=10; // placeholder
        public static final double turnGearing=10; // placeholder

}