package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class motorTest extends LinearOpMode {
    DcMotor motor;
    double curPower;
    @Override
    public void runOpMode() {
        curPower = 0.0;
        motor = hardwareMap.get(DcMotor.class, "motor");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.y) {
                curPower += 0.1;
                sleep(100);
            }
            if (gamepad1.a) {
                curPower -= 0.1;
                sleep(100);
            }
            if (gamepad1.right_bumper) {
                motor.setPower(curPower);
            }
            else {
                motor.setPower(0);
            }
            telemetry.addData("motorPower", curPower);
            telemetry.update();
        }
    }
}
