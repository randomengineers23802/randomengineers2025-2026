package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.base.RobotOpMode;
import org.firstinspires.ftc.teamcode.control.passthrough;

@Configurable
@TeleOp
public class teleOp extends RobotOpMode {
    private boolean shooting = false;
    private boolean endgame;
    private boolean prevRightTrigger = false;
    private boolean slowMode = false;
    private final double slowModeMultiplier = 0.25;

    @Override
    public void init() {
        super.init();
        if (passthrough.pose == null || passthrough.alliance == null) {
            telemetry.addLine("Run an auto first to set the pose and alliance");
        }
        else {
            follower.setStartingPose(passthrough.pose);
            robot.setAlliance(passthrough.alliance);
            telemetry.addData("Alliance", passthrough.alliance);
        }
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        robot.intake.off();
        robot.belt.off();
        robot.light.setColor(1.0);
    }

    @Override
    public void loop() {
        super.loop();
        if (!endgame) { robot.shooter.setVelocity(shotParameters.flywheelTicks + 20); }

        double speedMultiplier = slowMode ? slowModeMultiplier : 1.0;
        double x = gamepad1.left_stick_x * speedMultiplier;
        double y = gamepad1.left_stick_y * speedMultiplier;
        double turn = (gamepad1.left_trigger > 0.2) ? shotParameters.heading : -gamepad1.right_stick_x * speedMultiplier;
        follower.setTeleOpDrive(y, x, turn, false, robot.teleOpHeadingOffset);

        if (!shooting) {
            if (gamepad1.right_bumper) {
                robot.intake.on();
                robot.belt.onIntake();
            }
            else {
                robot.intake.off();
                robot.belt.off();
            }
        }

        boolean rightTriggerPressed = gamepad1.right_trigger > 0.2;
        boolean rightTriggerWasPressed = rightTriggerPressed && !prevRightTrigger;

        if (rightTriggerWasPressed && !shooting) {
            shootTimer.reset();
            shooting = true;
        }

        prevRightTrigger = rightTriggerPressed;

        if (shooting)
            Shoot();

        if (gamepad1.bWasPressed()) {
            shooting = false;
            robot.blueBoi.close();
            follower.startTeleopDrive();
        }

        if (gamepad1.xWasPressed() && follower.getVelocity().getMagnitude() < 1.5) {
            robot.relocalize();
        }

        if (gamepad1.yWasPressed()) {
            follower.followPath(robot.endgamePark.get());
            robot.shooter.off();
            slowMode = true;
            endgame = true;
        }

        if (gamepad1.dpadDownWasPressed()) {
            robot.kickstand.lower();
            robot.light.setColor(0.444);
        }
        else if (gamepad1.dpadUpWasPressed()) {
            robot.kickstand.raise();
            robot.light.setColor(1.0);
        }

        if (gamepad1.leftBumperWasPressed()) {
            slowMode = !slowMode;
        }
    }

    private void Shoot() {
        robot.belt.onShoot();
        robot.intake.on();
        double t = shootTimer.seconds();
        if (t <= 1.0)
            robot.blueBoi.open();
        else {
            robot.blueBoi.close();
            shooting = false;
        }
    }
}
