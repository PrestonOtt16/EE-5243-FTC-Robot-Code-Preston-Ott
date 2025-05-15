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

import java.util.Arrays;


//Op mode class
@Config
@TeleOp(name="motor2")
public class test2 extends OpMode {
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
    double errorx = 0;
    double error_prevx = 0;
    double errory = 0;
    double error_prevy = 0;
    double error_angle = 0;
    double error_prev_angle =0;
    //control signal
    public static double cs = 0.0;


    //x and y anf yaw position from odom pods/imu
    public static double x = 0;
    public static double y = 0;
    public static double yaw = 0.0;


    //changes in x and y
    public static double delta_x = 0.0;
    public static double delta_y = 0.0;
    public static double delta_yaw = 0.0;

    //radius from center
    double rx = 0.15;
    double ry = 0.17;

    //filter parameter
    int coolass_filter = 10;

    //Radius from wheel
    double r2 = 0.185;
    double r1 = 0.140;

    //proportional gain
    public static double kpx = 3.0;
    public static double kdx = -0.25;
    public static double kpy = 3.0;
    public static double kdy = -0.25;
    public static double kp_angle = 1.0;
    public static double kd_angle = -0.25;
    //derivatice gain
    public static double kd = -0.1;
    //target x = 1.0 meters
    public static double[] target = {0.0,0.0,0.0};


    public static double xdisp = 0.0;
    public static double ydisp = 0.0;

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
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD))
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

    //function for computing deltaxg
    public double deltaXg(double delta_x,double delta_y,double delta_yaw)
    {
        //adding d_yaw*rx too correct for rotation
        return (1-((delta_yaw*delta_yaw)/6))*delta_x - (delta_yaw/2)*delta_y - (delta_yaw*rx);
    }
    //function for computing deltayg
    public double deltaYg(double delta_x,double delta_y,double delta_yaw)
    {
        //adding d_yaw*ry too correct for rotation
        return (delta_yaw/2)*delta_x + (1-((delta_yaw*delta_yaw)/6))*delta_y + (delta_yaw*ry);
    }
    //function for computing delta_theta
    public double deltaYawg(double delta_yaw)
    {
        return delta_yaw;
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

            //computing start position
            //reading data from odometry pods
            //48mm diameter, 2000counts per revolution, 2pi*42[mm] per revolution
            //getting x2 and y2 and theta measurements
            theta1 = -frmotor.getCurrentPosition(); //x // reverse direction
            theta2 = flmotor.getCurrentPosition(); //y
            //computing x2,y2, I notice a (1/2) (2x x factor)(2.264 y factor) difference in odometry
            double xstart = (theta1 / 2000) * 2 * Math.PI * 0.024*(2);
            double ystart = (theta2 / 2000) * 2 * Math.PI * 0.024*(2);
            //getting the yaw
            double yawstart = getangle();

            //sending control signal

            //delay for 50ms
            try {
                 Thread.sleep(10);
            } catch (InterruptedException e) {
                    throw new RuntimeException(e);
            }



        //getting second position
            theta1 = -frmotor.getCurrentPosition(); //x // reverse direction
            theta2 = flmotor.getCurrentPosition(); //y
            //computing x2,y2
            double xstop = (theta1 / 2000) * 2 * Math.PI * 0.024*(2);
            double ystop = (theta2 / 2000) * 2 * Math.PI * 0.024*(2);
            double yawstop = getangle();

            //computing delta's
            delta_x = (xstop-xstart) ;
            delta_y = (ystop-ystart);
            delta_yaw = yaw_correct(yawstop,yawstart);

            //disp
            xdisp = delta_yaw*rx;
            ydisp = delta_yaw*ry;

            //updating x, y, yaw from delta g changes
            x = x + deltaXg(delta_x,delta_y,delta_yaw);
            y = y + deltaYg(delta_x,delta_y,delta_yaw);
            yaw = yaw + deltaYawg(delta_yaw);

            //computing xerror, yerror, xerror_prev, yerror_prev
            errorx = (target[0]-x);
            errory = (target[1]-y);
            error_angle = (target[2] - yaw);

            //applying proportional controller


            //moving robot using mechanum wheel equations, x p controller, y p controller, angle p controller
            flmotor.setPower((errorx)*kpx + (errorx-error_prevx)*kdx - ((errory*kpy)+(errory-error_prevy)*kdy) - (error_angle*kp_angle + error_prev_angle*kd_angle));
            blmotor.setPower((errorx)*kpx+ (errorx-error_prevx)*kdx + ((errory*kpy)+(errory-error_prevy)*kdy) - (error_angle*kp_angle)+ error_prev_angle*kd_angle);
            frmotor.setPower((errorx)*kpx+ (errorx-error_prevx)*kdx + ((errory*kpy)+(errory-error_prevy)*kdy) + (error_angle*kp_angle) + error_prev_angle*kd_angle);
            brmotor.setPower((errorx)*kpx + (errorx-error_prevx)*kdx - ((errory*kpy)+(errory-error_prevy)*kdy) + (error_angle*kp_angle) + error_prev_angle*kd_angle);

            //computing xerror_prev and yerror_prev for differential term
            error_prevx = errorx;
            error_prevy = errory;
            error_prev_angle = error_angle;



            //sending pose data
            telemetry.addData("delta_x", delta_x);
            telemetry.addData("delta_y", delta_y);
            telemetry.addData("delta_yaw", delta_yaw);

            //sending data on the movement of x and y from rotation
            telemetry.addData("xdisp",xdisp);
            telemetry.addData("ydisp",ydisp);

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("yaw", yaw);

            telemetry.addData("target",target);
            telemetry.addData("kpx",kpx);
            telemetry.addData("kpy",kpy);
            telemetry.addData("kp_angle",kp_angle);


    }
}