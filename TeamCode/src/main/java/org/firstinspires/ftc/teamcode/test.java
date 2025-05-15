//imports for test
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

//FTC dashboard imports
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



//imports for imu
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//import for thread
import java.lang.Thread;



//Op mode class
@Config
@TeleOp(name="motor1")
public class test extends OpMode {
    //motor instance
    DcMotor flmotor;
    DcMotor frmotor;
    DcMotor brmotor;
    DcMotor blmotor;

    //defining imu object
    public IMU imu1;

    //reading data from odometry pods
    float theta1 = 0; //fr motor
    float theta2 = 0; //fl motor


    //error pid controller
    public static double error = 0;
    double error_prev = 0;
    //error samples
    public static double error_angle = 0;
    //control signal
    public static double cs = 0.0;


    //x and y position from odom pods
    public static double x = 0;
    public static double y = 0;
    //proportional gain
    public static double kp = 1.0;
    //derivatice gain
    public static double kd = -0.1;
    public static double kp_angle = 1.0;
    public static double[] target = {0.0,0.0};
    //angle from imu
    public static double yaw = 0.0;


    //initialization function for robot
    @Override
    public void init() {
        flmotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        frmotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        //reverse right motors
        frmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brmotor = hardwareMap.get(DcMotor.class, "back_right_drive");
        //reverse right motors
        brmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blmotor = hardwareMap.get(DcMotor.class, "back_left_drive");

        //setting up imu in rev control hub
        imu1 = hardwareMap.get(IMU.class, "imu");
        //initalizing the imu object
        imu1.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,RevHubOrientationOnRobot.UsbFacingDirection.UP))
        );


        //creating a telemetry object
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    //function for imu reading
    public double getangle()
    {
        double angle = imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return angle;
    }


    //main loop for robot
    @Override
    public void loop() {

        //reading data from odometry pods
        theta1 = -frmotor.getCurrentPosition(); //x // reverse direction
        theta2 = flmotor.getCurrentPosition(); //y
        //48mm diameter, 2000counts per revolution, 2pi*42[mm] per revolution, negate y
        x = (theta1 / 2000) * 2 * Math.PI * 0.024;
        y = -(theta2 / 2000) * 2 * Math.PI * 0.024;
        //getting the yaw
        yaw = getangle();


        //error calculations
        error = Math.sqrt((target[0] - x)*(target[0]-x)+(target[1] - y)*(target[1]-y));
        double cs = (kp * error);

        //error2 calculations
        error_angle = (0 -yaw);
        double cs_angle = kp_angle*error_angle;


        //mechanum equations target[0] =y, target[1]= x, multiplied by control signal
        //apply angle component too mechanum equations
        flmotor.setPower(((target[0]-x) +(target[1] -y))*cs-cs_angle);
        blmotor.setPower(((target[0]-x)-(target[1]-y))*cs-cs_angle);
        frmotor.setPower(((target[0]-x)-(target[1]-y))*cs+cs_angle);
        brmotor.setPower(((target[0]-x) +(target[1] -y))*cs+cs_angle);

        // Using Telemetry object too update variables
        //telemetry.addData("target", target);
        telemetry.addData("x",x);
        telemetry.addData("y",y);
        telemetry.addData("yaw",yaw);
        telemetry.addData("error",error);
        telemetry.addData("cs",cs);
        telemetry.update();


    }
}