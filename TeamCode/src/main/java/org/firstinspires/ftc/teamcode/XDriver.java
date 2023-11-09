package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class XDriver{
    //declaring variables, for tweaking use sensitivity and top speed
    public DcMotorEx redMotor, blueMotor, greenMotor, yellowMotor;
    private BHI260IMU imu;

    public void init( HardwareMap hwMap )
    {
        BHI260IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.RADIANS,
                                0,
                                0,
                                0,
                                0  // acquisitionTime, not used
                        )
                )
        );

        imu = hwMap.get( BHI260IMU.class, "imu" );
        imu.initialize( parameters );
        imu.resetYaw();

        redMotor  = hwMap.get(DcMotorEx.class, "redMotor");
        greenMotor  = hwMap.get(DcMotorEx.class, "greenMotor");
        yellowMotor = hwMap.get(DcMotorEx.class, "yellowMotor");
        blueMotor = hwMap.get(DcMotorEx.class, "blueMotor");

        yellowMotor.setDirection(DcMotorEx.Direction.FORWARD);
        redMotor.setDirection(DcMotorEx.Direction.FORWARD);
        blueMotor.setDirection(DcMotorEx.Direction.REVERSE);
        greenMotor.setDirection(DcMotorEx.Direction.REVERSE);

        yellowMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        redMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        greenMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        blueMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    public double getHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetIMU(){
        imu.resetYaw();
    }

    public void drive(double angle, double power, double turn){

        double yellowPower = (Math.sin(angle + getHeading()) + Math.cos(angle + getHeading())) * power - turn;
        double greenPower =  (Math.sin(angle + getHeading()) + Math.cos(angle + getHeading())) * power + turn;
        double redPower = (Math.sin(angle + Math.PI / 2 + getHeading()) + Math.cos(angle + Math.PI / 2 + getHeading())) * power - turn;
        double bluePower = (Math.sin(angle + Math.PI / 2 + getHeading()) + Math.cos(angle + Math.PI / 2 + getHeading())) * power + turn;
        blueMotor.setPower(bluePower);
        redMotor.setPower(redPower);
        yellowMotor.setPower(yellowPower);
        greenMotor.setPower(greenPower);
    }
}