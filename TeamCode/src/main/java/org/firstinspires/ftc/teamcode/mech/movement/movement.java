package org.firstinspires.ftc.teamcode.mech.movement;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.mech.movement.goBuildaPinPointDriver.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.mech.movement.goBuildaPinPointDriver.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class movement {
    public GoBildaPinpointDriver odo;
    public wheel FL;
    public wheel FR;
    public wheel BL;
    public wheel BR;
    LinearOpMode li;
    public static double FR_PERCENT = 1;
    public static double BR_PERCENT = 1;
    public static double FL_PERCENT = 1.0;
    public static double BL_PERCENT = 1;
    public double power = 1.0;
    public static double x0;
    public static double y0;
    public static double h0;

    public movement(LinearOpMode l, double x, double y, double h){
        li = l;
        l.telemetry.addData("movement", "movement");
       /* odo = l.hardwareMap.get(GoBildaPinpointDriver.class, "imu");
        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.INCH, x, y,AngleUnit.DEGREES, h));

        */
        FL = new wheel(l.hardwareMap, "FL", false);
        FR = new wheel(l.hardwareMap, "FR", true);
        BL = new wheel(l.hardwareMap, "BL",false);
        BR = new wheel(l.hardwareMap, "BR", true);
        x0 = x;
        y0 = y;
        h0 = h;
    }
    public boolean is_busy(){
        return (FL.getPower() > 0 || FR.getPower() > 0 || BL.getPower() > 0  || BR.getPower() > 0 );
    }
    static double[] mixWheelPowers(double l_x, double l_y, double turn, double power) {
        power = Math.max(-1.0, Math.min(1.0, power));
        double fl = (+l_y - l_x - turn) * FL_PERCENT * power;
        double fr = (+l_y + l_x + turn) * FR_PERCENT * power;
        double bl = (+l_y + l_x - turn) * BL_PERCENT * power;
        double br = (+l_y - l_x + turn) * BR_PERCENT * power;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
        return new double[]{fl / max, fr / max, bl / max, br / max};
    }
    public void move(double l_x, double l_y, double turn){
        double[] wheelPowers = mixWheelPowers(l_x, l_y, turn, power);

        FL.setPower(wheelPowers[0]);
        FR.setPower(wheelPowers[1]);
        BL.setPower(wheelPowers[2]);
        BR.setPower(wheelPowers[3]);

      //  telemetry.addData("flpower", FLPower);
    }

}
