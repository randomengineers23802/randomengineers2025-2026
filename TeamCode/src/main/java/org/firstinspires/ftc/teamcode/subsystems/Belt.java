package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class Belt extends DcMotorImplEx {

    public Belt(HardwareMap hardwareMap) {
        this((DcMotorImplEx) hardwareMap.get(DcMotorEx.class, "belt"));
    }

    private Belt(DcMotorImplEx baseMotor) {
        super(baseMotor.getController(), baseMotor.getPortNumber(), baseMotor.getDirection());
        PIDFCoefficients beltPIDF = new PIDFCoefficients(0.0, 0.0, 0.0, 12.7);
        this.setDirection(DcMotor.Direction.REVERSE);
        this.setMode(RunMode.RUN_USING_ENCODER);
        this.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, beltPIDF);
    }

    public void onShoot() { this.setVelocity(1600); }
    public void onIntake() { this.setVelocity(2600); }
    public void off() { this.setPower(0.0); }
}