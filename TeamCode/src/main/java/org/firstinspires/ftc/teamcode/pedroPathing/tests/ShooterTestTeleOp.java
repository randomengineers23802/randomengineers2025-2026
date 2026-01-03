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

import org.firstinspires.ftc.teamcode.pedroPathing.customClasses.shooterControl;

@Configurable
@Disabled
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTestTeleOp extends OpMode {

    private shooterControl shooter;
    private TelemetryManager telemetryM;
    private Follower follower;

    private DcMotor belt;

    private DcMotor intake;

    private Servo BlueBoi = null;

    private ElapsedTime timer = new ElapsedTime();

    private boolean shooting = false;

    @Override
    public void init() {
        shooter = new shooterControl(hardwareMap, follower);
        belt = hardwareMap.get(DcMotor.class, "belt");
        belt.setDirection(DcMotor.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        BlueBoi = hardwareMap.get(Servo.class, "BlueBoi");
        BlueBoi.setPosition(0.65);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    private void Shoot() {
        belt.setPower(0.8);
        double t = timer.seconds();
        if (t <= 1.0)
            BlueBoi.setPosition(1.0);
        else {
            BlueBoi.setPosition(0.65);
            belt.setPower(0.0);
            shooting = false;
        }
    }

    @Override
    public void start() {
        belt.setPower(0.0);
    }

    @Override
    public void loop() {
        shooter.setShooterVelocity("close");
        if (gamepad1.right_trigger > 0.2 && !shooting) {
            timer.reset();
            shooting = true;
        }

        if (shooting)
            Shoot();

        if (gamepad1.left_trigger > 0.2) {
            belt.setPower(0.8);
        } else if (!shooting) {
            belt.setPower(0.0);
        }

        if (gamepad1.right_bumper) {
            intake.setPower(0.75);
        } else {
            intake.setPower(0.0);
        }

        telemetry.addData("Shooter L Velocity", shooter.ShooterL.getVelocity());
        telemetry.addData("Shooter R Velocity", shooter.ShooterR.getVelocity());
        telemetryM.debug("tpsL", shooter.ShooterL.getVelocity());
        telemetryM.debug("tpsR", shooter.ShooterR.getVelocity());
        telemetryM.addData("tpsL", shooter.ShooterL.getVelocity());
        telemetryM.addData("tpsR", shooter.ShooterR.getVelocity());
        telemetry.update();
        telemetryM.update();
    }
}
