package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;

public class swervedrive {
    public static final double kMaxSpeed=3.0; // 3 m/s
    // There should probably be a constant for these distance values otherwise it could be confusing.
    private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    public final swervemodule m_frontLeft = new swervemodule(1, 2, 0, 1, 2, 3);
    private final swervemodule m_frontRight = new swervemodule(3, 4, 4, 5, 6, 7);
    private final swervemodule m_backLeft = new swervemodule(5, 6, 8, 9, 10, 11);
    private final swervemodule m_backRight = new swervemodule(7, 8, 12, 13, 14, 15);

    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final SwerveDriveKinematics m_kinematics =
            new SwerveDriveKinematics(
                    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    private final SwerveDriveOdometry m_odometry =
            new SwerveDriveOdometry(
                    m_kinematics,
                    m_gyro.getRotation2d(),
                    new SwerveModulePosition[] {
                            m_frontLeft.getPosition(),
                            m_frontRight.getPosition(),
                            m_backLeft.getPosition(),
                            m_backRight.getPosition()
                    });
    //joystick info stuff
    public void drive(

            double xSpeed,double ySpeed,double rot, boolean fieldRelative, double periodSeconds
    ){
        var chassisSpeeds=new ChassisSpeeds(xSpeed,ySpeed,rot);
        var swerveModuleStates=m_kinematics.toWheelSpeeds(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,kMaxSpeed);
        m_frontLeft.SetDesired(swerveModuleStates[0]);
        m_frontRight.SetDesired(swerveModuleStates[1]);
        m_backLeft.SetDesired(swerveModuleStates[2]);
        m_backRight.SetDesired(swerveModuleStates[3]);

    }
}
