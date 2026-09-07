package frc.robot.subsystems.Shooter.Tigger;

import edu.wpi.first.units.measure.AngularVelocity;

public interface TriggerIO {

    public void setRPS(AngularVelocity RPS);

    public AngularVelocity getRPS();

    public boolean isAtSetPosition();

    public void stop();

    public boolean havefuel();
}