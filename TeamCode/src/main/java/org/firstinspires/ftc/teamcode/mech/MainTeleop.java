package org.firstinspires.ftc.teamcode.mech;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.mech.movement.goBuildaPinPointDriver.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.mech.movement.goBuildaPinPointDriver.Pose2D;
import org.firstinspires.ftc.teamcode.mech.movement.movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class MainTeleop extends LinearOpMode{
        private LinearOpMode lom;
        private movement movement;
        CRServo leftFlywheel, rightFlywheel;
        Servo spindexer;
        DcMotor leftIntake, rightIntake, launcher;
        double[] spindexerPosIntake = {0,0.38,0.79};
        double[] spindexerPosOuttake = {0.19,0.59,0.99};
        boolean intakeBool = true;
        int i = 0;
        @Override
        public void runOpMode() {
                //Initialize things here
                lom = this;
                movement = new movement(lom, 0, 0, 0);
                leftFlywheel = hardwareMap.get(CRServo.class, "leftFlywheel");
                rightFlywheel = hardwareMap.get(CRServo.class, "rightFlywheel");
                spindexer = hardwareMap.get(Servo.class, "spindexer");
                launcher = hardwareMap.get(DcMotor.class, "launcher");
                leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
                rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
                rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

                waitForStart();

                //Code here will run only once when Start is pressed

                while (opModeIsActive()) {

                        //Put any code here which should loop until Stop is pressed

                        boolean aButton = gamepad1.a;
                        boolean bButton = gamepad1.b;
                        boolean dLeft = gamepad1.dpad_left;
                        boolean dRight = gamepad1.dpad_right;
                        boolean lB = gamepad1.left_bumper;
                        boolean rB = gamepad1.right_bumper;
                        double x = gamepad1.left_stick_x;
                        double y = gamepad1.left_stick_y;
                        double h = gamepad1.right_stick_x;
                        movement.move(x,y,h);
                       // movement = new movement(lom, x, y, h);

                        if (gamepad1.y) {
                                launcher.setPower(1);
                                sleep(2000);
                                leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
                                leftFlywheel.setPower(1);
                                rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                                rightFlywheel.setPower(1);
                        }
                        if (gamepad1.a) {
                                leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                                leftFlywheel.setPower(0);
                                rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
                                rightFlywheel.setPower(0);
                                launcher.setPower(0);
                        }
                        if (dLeft&&i<spindexerPosIntake.length) {
                                i++;
                                sleep(200);
                        }
                        if (dRight&&i>0) {
                                i--;
                                sleep(200);
                        }
                        if (rB) {
                                rightIntake.setPower(1);
                                leftIntake.setPower(1);
                                intakeBool = true;
                        }
                        if (lB) {
                                rightIntake.setPower(-1);
                                leftIntake.setPower(-1);
                                intakeBool = true;
                        }
                        intakeBool = false;
                        leftIntake.setPower(0);
                        rightIntake.setPower(0);
                        if (intakeBool) {
                                spindexer.setPosition(spindexerPosIntake[i]);
                        }
                        else {
                                spindexer.setPosition(spindexerPosOuttake[i]);
                        }
                        idle(); //Give the system more time to do background tasks
                        //This shouldn't be necessary and isn't in the boilerplate template,
                        //but try adding it if your program crashes at random just in case.
                        telemetry.addData("x", x);
                        telemetry.addData("y", y);
                        telemetry.addData("h", h);
                        telemetry.update(); //OpMode does this for you

                }

                //Put anything here that needs to run once when Stop is pressed

        }
}
