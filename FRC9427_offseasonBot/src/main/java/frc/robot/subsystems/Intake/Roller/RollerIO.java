package frc.robot.subsystems.Intake.Roller;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface RollerIO {

    public void setRPS(AngularVelocity RPS);

    public AngularVelocity getRPS();

    public boolean isAtSetPosition();

    public void stop();
}