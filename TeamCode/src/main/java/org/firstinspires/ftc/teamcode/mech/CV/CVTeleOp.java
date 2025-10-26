package org.firstinspires.ftc.teamcode.mech.CV;
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
public class CVTeleOp extends LinearOpMode{
    private LinearOpMode lom;
    private CV cv;

    @Override
    public void runOpMode() {
        lom = this;
        cv = new CV(lom);
        waitForStart();

        //Code here will run only once when Start is pressed

        while (opModeIsActive()) {
            cv.stream();
        }
    }
}
