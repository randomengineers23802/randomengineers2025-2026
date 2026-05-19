package org.firstinspires.ftc.teamcode.base;

public abstract class AutoOpMode extends RobotOpMode {
    protected int pathState = 0;

    @Override
    public void start() {
        robot.intake.on();
        robot.belt.onShoot();
        robot.blueBoi.close();
    }

    protected void Shoot() {
        if (!follower.isBusy()) {
            if (shootTimer.seconds() <= 1.0) {
                robot.blueBoi.open();
            } else {
                robot.blueBoi.close();
                pathState++;
                shootTimer.reset();
            }
        } else {
            shootTimer.reset();
        }
    }
}
