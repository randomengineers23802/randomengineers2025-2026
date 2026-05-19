package org.firstinspires.ftc.teamcode.base;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.control.RobotControl;
import org.firstinspires.ftc.teamcode.control.ShotParameters;
import org.firstinspires.ftc.teamcode.control.passthrough;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
public abstract class RobotOpMode extends OpMode {
    protected RobotControl robot;
    protected Follower follower;
    protected TelemetryManager panelsTelemetry;
    protected ElapsedTime shootTimer = new ElapsedTime();
    protected ShotParameters shotParameters;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        robot = new RobotControl(hardwareMap, follower);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.update(telemetry);
        robot.kickstand.raise();
    }

    @Override
    public void loop() {
        follower.update();
        shotParameters = robot.updateShooting();
    }

    @Override
    public void stop() {
        passthrough.pose = follower.getPose();
    }
}
