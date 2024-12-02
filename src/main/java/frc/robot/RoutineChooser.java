package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autos.bottomPathClass;
import frc.robot.autos.middlePathClass;
import frc.robot.autos.taxiPathClass;

public class RoutineChooser {
    private static final String bottomPathClass = "bottomPath"; // replace fillers with filenames of the autos, the strings are the assigned auto names
    private static final String middlePathClass = "middlePath";
    private static final String taxiPathClass = "taxiPath";
    bottomPathClass bottomPath = new bottomPathClass();
    middlePathClass middlePath = new middlePathClass();
    taxiPathClass taxiPath = new taxiPathClass();
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // This should be called in the robot initialization to initialize autos.
    public void InitializeAutos() {
        m_chooser.setDefaultOption("filler1", taxiPathClass);
        m_chooser.addOption("middlePath", middlePathClass);
        m_chooser.addOption("bottom", bottomPathClass);
        SmartDashboard.putData("Auto Choices", m_chooser);
    }

    // Called in the initialization of the autos on the robots.
    public void autonomousAutoInit() {
        m_autoSelected = m_chooser.getSelected();
        System.out.println("Auto selected: " + m_autoSelected);
    }
    public void autonomousPeriodic() {
        switch (m_autoSelected) {
            case bottomPathClass:
                bottomPath.bottomPath();
                break;
            case middlePathClass:
                middlePath.middlePath();
                break;
            case taxiPathClass:
            default:
                taxiPath.taxiPath();
                break;
        }
    }
}


