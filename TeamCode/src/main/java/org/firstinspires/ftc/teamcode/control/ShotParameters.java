package org.firstinspires.ftc.teamcode.control;

public class ShotParameters {
    public final double flywheelTicks;
    public final double heading;

    public ShotParameters(double flywheelSpeed, double heading) {
        this.flywheelTicks = robotControl.getFlywheelTicksFromVelocity(flywheelSpeed);
        this.heading = heading;
    }
}