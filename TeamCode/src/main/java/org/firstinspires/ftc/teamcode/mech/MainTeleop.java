package org.firstinspires.ftc.teamcode.mech;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.mech.CV.ColorDetection;
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
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.mech.CV.ColorDetection;

@TeleOp
public class MainTeleop extends LinearOpMode{
        private LinearOpMode lom;
        private movement movement;
        CRServo leftFlywheel, rightFlywheel;
        ColorSensor sensor;
        Servo spindexer;
        DcMotor leftIntake, rightIntake, launcher;
        double[] spindexerPosIntake = {0,0.38,0.79};
        double[] spindexerPosOuttake = {0.19,0.59,0.99};
        boolean intakeBool = false;
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
                List<String> ballcols = new ArrayList<>();
                        ballcols.add("blank");
                        ballcols.add("blank");
                        ballcols.add("blank");
                sensor = hardwareMap.get(ColorSensor.class, "colorSensor");
                ColorDetection colorSensor = new ColorDetection();
                boolean patternchecked = false;
                int p = 0;
                String color = "blank";

                waitForStart();

                //Code here will run only once when Start is pressed

                while (opModeIsActive()) {

                        //Put any code here which should loop until Stop is presses
                        boolean dLeft = gamepad1.dpad_left;
                        boolean dRight = gamepad1.dpad_right;
                        boolean lB = gamepad1.left_bumper;
                        boolean rB = gamepad1.right_bumper;
                        double x = gamepad1.left_stick_x;
                        double y = gamepad1.left_stick_y;
                        double h = gamepad1.right_stick_x;
                        movement.move(x,y,h);

                       // movement = new movement(lom, x, y, h);

//                        if (gamepad1.y) {
//                                launcher.setPower(1);
//                                sleep(2000);
//                                leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
//                                leftFlywheel.setPower(1);
//                                rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
//                                rightFlywheel.setPower(1);
//                        }
//                        if (gamepad1.a) {
//                                leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
//                                leftFlywheel.setPower(0);
//                                rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
//                                rightFlywheel.setPower(0);
//                                launcher.setPower(0);
//                        }

                        if (gamepad1.x && !patternchecked) {
                                p = 0;
                                patternchecked=true;
                        }
                        if (gamepad1.y && !patternchecked) {
                                p = 1;
                                patternchecked=true;
                        }
                        if (gamepad1.b && !patternchecked) {
                                p = 2;
                                patternchecked=true;
                        }

                         if (gamepad1.y && patternchecked) {
                                // y is basically green purple purple as a sequence
                                List<Integer> poslist = new ArrayList<>();
                                        poslist.add(0);
                                        poslist.add(0);
                                        poslist.add(0);
                                boolean green = false;
                                int purple = 0;
                                for (int j = 0; j < 2; j++) {
                                        telemetry.addData("j", "This works");
                                        color = ballcols.get(j);
                                        if ("green".equals(color) && !green) {
                                                poslist.set(p,j);
                                                poslist.set((p+1)%3,(j+1)%3);
                                                poslist.set((p+2)%3,(j+2)%3);
                                                green = true;
                                        } else if (("green".equals(color) && green) || "blank".equals(color) || "purple".equals(color) && purple > 2) {
                                                telemetry.addData("blank", "This works");
                                                poslist.set(0,67);
                                        }
                                        else {
                                              purple = purple + 1;
                                        }

                                }
                                if (poslist.get(0) == 67){
                                        randomrunner:
                                        for (int j = 0; j < 3; j++){
                                                spindexer.setPosition(spindexerPosOuttake[j]);
                                                launcher.setPower(1);
                                                if (j == 0){
                                                        sleep(1500);
                                                }
                                                if(gamepad1.a){
                                                        break randomrunner;
                                                }
                                                leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
                                                leftFlywheel.setPower(1);
                                                rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                                                rightFlywheel.setPower(1);
                                                sleep(750);
                                                if(gamepad1.a){
                                                        break randomrunner;
                                                }
                                                leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                                                leftFlywheel.setPower(0);
                                                if(gamepad1.a){
                                                        break randomrunner;
                                                }
                                                rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
                                                rightFlywheel.setPower(0);
                                                launcher.setPower(0);
                                                if(gamepad1.a){
                                                        break randomrunner;
                                                }
                                        }
                                }
                                else{
                                        properrunner:
                                        for (int j = 0; j < 3; j++){
                                                spindexer.setPosition(spindexerPosOuttake[poslist.get(j)]);
                                                launcher.setPower(1);
                                                if(gamepad1.a){
                                                        break properrunner;
                                                }
                                                if (j == 0){
                                                        sleep(1500);
                                                }
                                                if(gamepad1.a){
                                                        break properrunner;
                                                }
                                                leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
                                                leftFlywheel.setPower(1);
                                                rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                                                if(gamepad1.a){
                                                        break properrunner;
                                                }
                                                rightFlywheel.setPower(1);
                                                sleep(500);
                                                if(gamepad1.a){
                                                        break properrunner;
                                                }
                                                leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                                                leftFlywheel.setPower(0);
                                                rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
                                                rightFlywheel.setPower(0);
                                                launcher.setPower(0);
                                                if(gamepad1.a){
                                                        break properrunner;
                                                }
                                        }
                                }
                        }
                        if (dLeft&&i<spindexerPosIntake.length-1) {
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
                                ballcols.set(i, colorSensor.getColor(sensor));
                        }
                        else if (lB) {
                                rightIntake.setPower(-1);
                                leftIntake.setPower(-1);
                                intakeBool = true;
                                ballcols.set(i, colorSensor.getColor(sensor));
                        }
                        else {
                                rightIntake.setPower(0);
                                leftIntake.setPower(0);
                        }
                        telemetry.addData("spindexerPosIntake", spindexerPosOuttake[i]);
                        if (intakeBool) {
                                spindexer.setPosition(spindexerPosIntake[i]);
                        }
                        idle(); //Give the system more time to do background tasks
                        //This shouldn't be necessary and isn't in the boilerplate template,
                        //but try adding it if your program crashes at random just in case.
                        telemetry.addData("x", x);
                        telemetry.addData("y", y);
                        telemetry.addData("h", h);
                        telemetry.addData("intakeBool", intakeBool);
                        telemetry.addData("i", i);
                        telemetry.addData("spindexerPos", spindexer.getPosition());
                        telemetry.addData("spindexerPosOuttake", spindexerPosOuttake[i]);
                        telemetry.update(); //OpMode does this for you

                }

                //Put anything here that needs to run once when Stop is pressed

        }
}
