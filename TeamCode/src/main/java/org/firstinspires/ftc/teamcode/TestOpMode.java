package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="Test Opmode", group="Iterative OpMode")
public class TestOpMode extends OpMode
{
    public static XDriver robot;
    public static double p, i, d, f;
    public static double targetMotorVelocity;

    @Override
    public void init() {
        robot = new XDriver();
        robot.init(hardwareMap);
        robot.redMotor.setVelocityPIDFCoefficients(p, i, d, f);
        robot.greenMotor.setVelocityPIDFCoefficients(p, i, d, f);
        robot.blueMotor.setVelocityPIDFCoefficients(p, i, d, f);
        robot.yellowMotor.setVelocityPIDFCoefficients(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        robot.redMotor.setVelocityPIDFCoefficients(p, i, d, f);
        robot.greenMotor.setVelocityPIDFCoefficients(p, i, d, f);
        robot.blueMotor.setVelocityPIDFCoefficients(p, i, d, f);
        robot.yellowMotor.setVelocityPIDFCoefficients(p, i, d, f);

        robot.drive(Math.toRadians(45.0), targetMotorVelocity / 2800.0, 0);

        double redVel = robot.redMotor.getVelocity();
        double greenVel = robot.greenMotor.getVelocity();
        double blueVel = robot.blueMotor.getVelocity();
        double yelVel = robot.yellowMotor.getVelocity();
        telemetry.addData("red vel", redVel);
        telemetry.addData("green Vel", greenVel);
        telemetry.addData("blue Vel", blueVel);
        telemetry.addData("yel Vel", yelVel);
        telemetry.addData("target vel", targetMotorVelocity);
        telemetry.update();
    }
}
