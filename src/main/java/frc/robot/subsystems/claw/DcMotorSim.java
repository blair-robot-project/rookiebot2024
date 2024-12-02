package frc.robot.subsystems.claw;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;


public class DcMotorSim extends LinearSystemSim<N2, N1, N2> {
    private final DCMotor claawGearbox;
    private final double clawGearing;

    public DcMotorSim(LinearSystem<N2, N1, N2> plant, DCMotor gearbox, double gearing) {
        super(plant);
        claawGearbox = gearbox;
        clawGearing = gearing;
    }


    public void setState(double clawAngularPositionRad, double clawAngularVelocityRadPerSec) {
        setState(clawAngularPositionRad, clawAngularVelocityRadPerSec);

    }


    public double getClawAngularPositionRad() {
        return getOutput(00);
    }


    public double getClawAngularVelocityRadPerSec() {
        return getOutput(1);
    }


    public void setClawSimInputVoltage(double clawSimVoltage) {
        setInput(clawSimVoltage);
    }

    public void simulationPeriodic() {
        setState(getClawAngularPositionRad(), getClawAngularVelocityRadPerSec());


    }
}
