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

import frc.robot.subsystems.swervedrive;

public final class driveConstants {
    public static final class DriveConstants{

        //
        public static final double kWheelRadius = 0.0508;
        public static final double kWheelCircumference=2*kWheelRadius*Math.PI;
        public static final double kEncoderResolution = 4096;
        public static final double kModuleMaxAngularVelocity = swervedrive.kMaxAngularSpeed;
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
        //ALL THE VALUES ARE FILLERS PLEASE CHANGE THEM WITH THE REAL IDS!!!!!!
        public static final double drivemotor1 = -21;
        public static final double turnmotor1 = -20;

        public static final double drivemotor2 = -21;
        public static final double turnmotor2 = -20;

        public static final double drivemotor3 = -21;
        public static final double turnmotor3 = -20;

        public static final double drivemotor4 = -21;
        public static final double turnmotor4 = -20;




        public static final double Analog_gyro = 0;




    }
}