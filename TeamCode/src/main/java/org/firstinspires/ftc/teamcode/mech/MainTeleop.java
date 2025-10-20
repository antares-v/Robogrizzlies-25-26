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




@TeleOp
public class MainTeleop extends LinearOpMode{
        private LinearOpMode lom;
        private movement movement;
        private double x, y, z;
        @Override
        public void runOpMode() {
                //Initialize things here
                lom = this;
                movement = new movement(lom, x, y, z);

                waitForStart();

                //Code here will run only once when Start is pressed

                while (opModeIsActive()) {

                        //Put any code here which should loop until Stop is pressed

                        x = gamepad1.left_stick_x;
                        y = gamepad1.left_stick_y;
                        z = gamepad1.right_stick_y;
                        movement.move(x,y,z);

                        idle(); //Give the system more time to do background tasks
                        //This shouldn't be necessary and isn't in the boilerplate template,
                        //but try adding it if your program crashes at random just in case.

                        telemetry.update(); //OpMode does this for you
                }

                //Put anything here that needs to run once when Stop is pressed

        }
}
