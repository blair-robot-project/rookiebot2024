package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.bottomPathClass;
import frc.robot.autos.middlePathClass;
import frc.robot.autos.taxiPathClass;

public class RoutineChooser {
    bottomPathClass bottomPath = new bottomPathClass();
    middlePathClass middlePath = new middlePathClass();
    taxiPathClass taxiPath = new taxiPathClass();
    private String m_autoSelected;
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    // This should be called in the robot initialization to initialize autos.
    public void InitializeAutos() {
        m_chooser.setDefaultOption("taxiPath", taxiPath.taxiPath());
        m_chooser.addOption("middlePath", middlePath.middlePath());
        m_chooser.addOption("bottom", bottomPath.bottomPath());
        SmartDashboard.putData("Auto Choices", m_chooser);
    }



    // Called in the initialization of the autos on the robots.
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

}


