package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.robotControl;

@Configurable
@Disabled
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTestTeleOp extends OpMode {

    private robotControl robot;
    private TelemetryManager telemetryM;
    private Follower follower;

    private DcMotor belt;

    private DcMotor intake;

    private Servo BlueBoi = null;

    private ElapsedTime timer = new ElapsedTime();

    private boolean shooting = false;

    @Override
    public void init() {
        robot = new robotControl(hardwareMap, follower);
        belt = hardwareMap.get(DcMotor.class, "belt");
        belt.setDirection(DcMotor.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        BlueBoi = hardwareMap.get(Servo.class, "BlueBoi");
        robot.blueBoiClosed();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    private void Shoot() {

        double t = timer.seconds();
        if (t <= 1.0)
            robot.blueBoiOpen();
        else {
            robot.blueBoiClosed();

            shooting = false;
        }
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robot.setShooterVelocity("close");
        if (gamepad1.right_trigger > 0.2 && !shooting) {
            timer.reset();
            shooting = true;
        }

        if (shooting)
            Shoot();

        if (gamepad1.left_trigger > 0.2) {

        } else if (!shooting) {

        }

        if (gamepad1.right_bumper) {
            intake.setPower(0.75);
        } else {
            robot.intakeOff();
        }

        telemetry.addData("Shooter L Velocity", robot.ShooterL.getVelocity());
        telemetry.addData("Shooter R Velocity", robot.ShooterR.getVelocity());
        telemetryM.debug("tpsL", robot.ShooterL.getVelocity());
        telemetryM.debug("tpsR", robot.ShooterR.getVelocity());
        telemetryM.addData("tpsL", robot.ShooterL.getVelocity());
        telemetryM.addData("tpsR", robot.ShooterR.getVelocity());
        telemetry.update();
        telemetryM.update();
    }
}
