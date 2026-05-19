package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends DcMotorImpl {

    public Intake(HardwareMap hardwareMap) {
        this((DcMotorImpl) hardwareMap.get(DcMotor.class, "intake"));
    }

    private Intake(DcMotorImpl baseMotor) {
        super(baseMotor.getController(), baseMotor.getPortNumber());
        this.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.setDirection(DcMotor.Direction.REVERSE);
    }

    public void on() {
        this.setPower(1.0);
    }

    public void off() {
        this.setPower(0.1);
    }
}