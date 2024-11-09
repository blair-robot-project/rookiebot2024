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

        public static final double kWheelRadius = 0.0508;
        public static final double kWheelCircumference=2*kWheelRadius*Math.PI;
        public static final double kEncoderResolution = 4096;
        public static final double kModuleMaxAngularVelocity = swervedrive.kMaxAngularSpeed;
        public static final double kModuleMaxAngularAcceleration = 2 * Math.PI;

      //motor ids
        public static final double drivemotor1 = 21;
        public static final double m_frontLeft = 20;

        //swerve module channels
        //pls replace the values with real ones
        public static final double drivemotor = 1;
        public static final double turnmotor = 2;
        public static final double turn_encoder1 = 3;





    }
}