package org.firstinspires.ftc.teamcode.mech;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.mech.movement.goBuildaPinPointDriver.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.mech.movement.goBuildaPinPointDriver.Pose2D;
import org.firstinspires.ftc.teamcode.mech.movement.movement;

import org.firstinspires.ftc.teamcode.mech.intake.intake;

import org.firstinspires.ftc.teamcode.mech.mainIntakeOuttake.launcher;
import org.firstinspires.ftc.teamcode.mech.mainIntakeOuttake.rotatingWheel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;





@TeleOp
public class MainOp extends LinearOpMode{
        private LinearOpMode lom;
        private movement movement;
        private launcher launcher1;
        private launcher launcher2;
        private intake intake;
        private rotatingWheel rotatingWheel;
        @Override
        public void runOpMode() {
                //Initialize things here
                lom = this;
                movement = new movement(lom, 0, 0, 0);
                intake = new intake(lom, false);
                launcher1 = new launcher(lom,"launcher1");
                launcher2 = new launcher(lom,"launcher2");
                rotatingWheel = new rotatingWheel(lom);
          
                waitForStart();

                //Code here will run only once when Start is pressed

                while (opModeIsActive()) {

                        //Put any code here which should loop until Stop is pressed

                        double x = gamepad1.left_stick_x;
                        boolean aButton = gamepad1.a;
                        boolean bButton = gamepad1.b;
                        boolean xButton = gamepad1.x;
                        boolean yButton = gamepad1.y;
                        double y = gamepad1.left_stick_y;
                        double h = gamepad1.right_stick_x;
                        movement.move(x,y,h);

                        if(aButton){
                          intake.takeIn();
                        } else if(bButton){
                          intake.eject();
                        }else{
                          intake.stop();
                        } 

                        if(xButton){
                                rotatingWheel.servopos1();    
                        } else if(rotatingWheel.location()!=2){
                                rotatingWheel.servopos2();
                        }
                        if(yButton){
                                launcher1.on();
                                launcher2.on();
                        } else{
                                launcher1.off();
                                launcher2.off();
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
