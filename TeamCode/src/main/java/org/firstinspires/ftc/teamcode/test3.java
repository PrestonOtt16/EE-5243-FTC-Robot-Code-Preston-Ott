//imports for test
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//array and thread libraries
import java.util.Arrays;
import java.lang.Thread;


//Op mode class
@Config
@TeleOp(name="motor4")
public class test3 extends OpMode {
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


    //x and y anf yaw position from odom pods/imu
    public static double x = 0;
    public static double y = 0;
    public static double yaw = 0.0;
    public static double theta =0.0;
    public static double R = 0;
    public static double dx = 0;
    public static double dy = 0;


    //radius from center
    double rx = 0.15;
    double ry = 0.17;


    //Radius from wheel
    double r2 = 0.185;
    double r1 = 0.140;


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
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD))
        );


        //creating a telemetry object
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    //function for imu reading
    public double getangle() {
        double angle = imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return angle;
    }

    //yaw absolute correction function
    public double yaw_correct(double yaw2,double yaw1)
    {
        if(yaw2 - yaw1 > Math.PI)
        {
            return -Math.PI*2 +(yaw2-yaw1);
        }
        else if(yaw2 - yaw1 < -Math.PI)
        {
            return Math.PI*2+(yaw2-yaw1);
        }
        else
        {
            return (yaw2-yaw1);
        }
    }


    //main loop for robot
    @Override
    public void loop() {

        //computing x1,y1
        theta1 = -frmotor.getCurrentPosition(); //x // reverse direction
        theta2 = flmotor.getCurrentPosition(); //y
        double x1 = (theta1 / 2000) * 2 * Math.PI * 0.024;
        double y1 = (theta2 / 2000) * 2 * Math.PI * 0.024;
        //getting the yaw
        double yaw1 = getangle();

        //sending control signal

        //delaying for 50ms = delta_t
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        //computing x2,y2
        theta1 = -frmotor.getCurrentPosition(); //x // reverse direction
        theta2 = flmotor.getCurrentPosition(); //y
        double x2 = (theta1 / 2000) * 2 * Math.PI * 0.024;
        double y2 = (theta2 / 2000) * 2 * Math.PI * 0.024;
        //getting the yaw
        double yaw2 = getangle();

        //applying yaw absolute correction function

        //calculating theta and R

        R = 0;
        dx = (x2 - x1);
        dy = (y2 - y1);
        theta = yaw_correct(yaw2,yaw1);
        double theta_min = 0.001;
/*
        if(theta > theta_min)
        {
            R = ((x2-x1)/(theta))+rx;
            //calculating dx and dy from arc movement
            dx = Math.cos(yaw1)*R*Math.sin(theta)-Math.sin(yaw1)*R*Math.cos(theta);
            dy = Math.sin(yaw1)*R*Math.sin(theta)+Math.cos(yaw1)*R*Math.cos(theta);
        }
        if(theta < -theta_min)
        {
            R = ((x2-x1)/(theta))-rx;
            //calculating dx and dy from arc movement
            dx = Math.cos(yaw1)*R*Math.sin(theta)-Math.sin(yaw1)*R*Math.cos(theta);
            dy = Math.sin(yaw1)*R*Math.sin(theta)+Math.cos(yaw1)*R*Math.cos(theta);
        }
        else
        {
            //calculating dx and dy from arc movement
            dx = (x2 - x1);
            dy = 0;
        }

*/
        //updating the x,y,yaw position
        x = x+dx;
        y = y+dy;
        yaw = yaw+theta;



        //sending pose data
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("yaw",yaw);
        //telemetry.addData("R",R);
        telemetry.update();


    }
}
