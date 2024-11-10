/**package frc.robot.allConstants.driveConstants

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swervedrive;

object driveConstants{
  const val kWheelRadius = 0.0508;
  const val kWheelCircumference=2*kWheelRadius*Math.PI;
  const val kEncoderResolution = 4096;
  const val kModuleMaxAngularVelocity = swervedrive.kMaxAngularSpeed;
  const val kModuleMaxAngularAcceleration = 2 * Math.PI;

// the locations of the wheels
  const val m_frontLeftLocation = (0.381, 0.381);
  const val m_frontRightLocation = (0.381, -0.381)
  const val m_backLeftLocation = (-0.381, 0.381);
  const val m_backRightLocation = (-0.381, -0.381);
}
**/

package frc.robot.allConstants;
import frc.robot.subsystems.swerveDrive;
public final class driveConstants {

        public static final double kWheelRadius = 0.0508;
        public static final double kWheelCircumference=2*kWheelRadius*Math.PI;
        public static final double kEncoderResolution = 4096;
        public static final double kModuleMaxAngularVelocity = swerveDrive.kMaxAngularSpeed;
        public static final double kModuleMaxAngularAcceleration = 2 * Math.PI;

        //location of the wheels at the start
        public static final double frontLeftLocationX = 0.381;
        public static final double frontLeftLocationY = 0.381;

        public static final double frontRightLocationX = 0.381;
        public static final double frontRightLocationY = -0.381;

        public static final double backLeftLocationX = -0.381;
        public static final double backLeftLocationY = 0.381;

        public static final double backRightLocationX = -0.381;
        public static final double backRightLocationY = -0.381;

        //PID calculator values for swerve

        public static final double drivePIDkp = 0.25;
        public static final double drivePIDki = 0.0;
        public static final double drivePIDkd = 0.0;

        public static final double turnPIDkp = 0.25;
        public static final double turnPIDki = 0.0;
        public static final double turnPIDkd = 0.0;

        //feedForward calc values for swerve

        public static final double swerveFeedForwardDriveks = 1;
        public static final double swerveFeedForwardDrivekv = 1;
        public static final double swerveFeedForwardDriveka = 1;

        public static final double swerveFeedForwardTurnks = 1;
        public static final double swerveFeedForwardTurnkv = 1;
        public static final double swerveFeedForwardTurnka = 1;

        //8 motor ids
        public static final int driveMotor1 = 20;
        public static final int turnMotor1 = 21;

        public static final int driveMotor2 = 17;
        public static final int turnMotor2 = 61;

        public static final int driveMotor3 = 1;
        public static final int turnMotor3 = 2;

        public static final int driveMotor4 = 4;
        public static final int turnMotor4 = 3;

        //the interavals in which the motor loops
        public static final double pdsec = 0.02;

        //no matter the robot position forward is forward (frfr = field relative:)
        public static final boolean frfr = true;


        public static final double kMaxSpeed=3.0; // 3 m/s
        public static final double kMaxAngularSpeed=Math.PI; // 1/2 rotation per second (in radians, so pi radians is 1/2 of a rotation)

        //don't really know what this is or what it does
        public static final double Analog_gyro = 0;


}