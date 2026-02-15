package org.firstinspires.ftc.teamcode.control;

import static org.firstinspires.ftc.teamcode.control.robotControl.getFlywheelTicksFromVelocity;

public class ShotParameters {
    public final double flywheelTicks;
    public final double hoodAngle;
    public final double turretAngle;

    public ShotParameters(double flywheelTicks, double hoodAngle, double turretAngle) {
        this.flywheelTicks = getFlywheelTicksFromVelocity(flywheelTicks);
        this.hoodAngle = Math.toDegrees(hoodAngle);
        this.turretAngle = turretAngle;
    }
}
