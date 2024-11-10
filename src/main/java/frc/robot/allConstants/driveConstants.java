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
        public static final double frontLeftLocationx = 0.381;
        public static final double frontLeftLocationy = 0.381;

        public static final double frontRightLocationx = 0.381;
        public static final double frontRightLocationy = -0.381;

        public static final double backLeftLocationx = -0.381;
        public static final double backLeftLocationy = 0.381;

        public static final double backRightLocationx = -0.381;
        public static final double backRightLocationy = -0.381;



        //8 motor ids
        public static final int drivemotor1 = 20;
        public static final int turnmotor1 = 21;

        public static final int drivemotor2 = 17;
        public static final int turnmotor2 = 61;

        public static final int drivemotor3 = 1;
        public static final int turnmotor3 = 2;

        public static final int drivemotor4 = 4;
        public static final int turnmotor4 = 3;

        //the interavals in which the motor loops
        public static final double pdsec = 0.02;

        //no matter the robot position forward is forward (frfr = field relative:)
        public static final boolean frfr = true;


        public static final double kMaxSpeed=3.0; // 3 m/s
        public static final double kMaxAngularSpeed=Math.PI; // 1/2 rotation per second (in radians, so pi radians is 1/2 of a rotation)

        //don't really know what this is or what it does
        public static final double Analog_gyro = 0;


}