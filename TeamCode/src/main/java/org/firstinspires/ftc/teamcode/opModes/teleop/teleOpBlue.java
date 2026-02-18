package org.firstinspires.ftc.teamcode.opModes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.control.passthrough;
import org.firstinspires.ftc.teamcode.control.robotControl;

@Configurable
@TeleOp(name = "teleOpBlue", group = "TeleOp")
public class teleOpBlue extends OpMode {
    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private boolean slowMode = false;
    private final double slowModeMultiplier = 0.25;
    private robotControl robot;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(passthrough.startPose);
        follower.update();
        robot = new robotControl(hardwareMap, follower, gamepad1);
        robot.setAlliance("blue");
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        robot.updateTurret();
        panelsTelemetry.addData("Shooter1", robot.Shooter1.getVelocity());
        panelsTelemetry.addData("Shooter2", robot.Shooter2.getVelocity());
        panelsTelemetry.update();

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        if (!slowMode) {
            follower.setTeleOpDrive(y, x, turn, false);
        } else {
            follower.setTeleOpDrive(
                    y * slowModeMultiplier,
                    x * slowModeMultiplier,
                    turn * slowModeMultiplier,
                    false
            );
        }

        robot.relocalize(); // only happens if x is pressed

        if (gamepad1.leftBumperWasPressed()) {
            slowMode = !slowMode;
        }
    }

    @Override
    public void stop() {
        passthrough.startPose = follower.getPose();
    }
}