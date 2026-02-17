package org.firstinspires.ftc.teamcode.control;

public class ShotParameters {
    public final double flywheelTicks;
    public final double hoodPosition;
    public final double turretAngle;

    public ShotParameters(double flywheelSpeed, double hoodDegrees, double turretAngle) {
        this.flywheelTicks = robotControl.getFlywheelTicksFromVelocity(flywheelSpeed);
        this.hoodPosition = robotControl.getHoodPositionFromDegrees(hoodDegrees);
        this.turretAngle = turretAngle;
    }
}