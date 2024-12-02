package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RoutineChooser {
    private static final String bottomPathClass = "bottomPath"; // replace fillers with filenames of the autos, the strings are the assigned auto names
    private static final String middlePathClass = "middlePath";
    private static final String taxiPathClass = "taxiPath";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public void InitializeAutos() {
        m_chooser.setDefaultOption("filler1", taxiPathClass);
        m_chooser.addOption("middlePath", middlePathClass);
        m_chooser.addOption("bottom", bottomPathClass);
        m_chooser.addOption("taxiPath", taxiPathClass);
        SmartDashboard.putData("Auto Choices", m_chooser);
    }
}

