package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;

@TeleOp(name = "decodeTeleop", group = "Iterative OpMode")

public class decodeTeleop extends BasicOpMode_Iterative {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    double frontLeftPower, frontRightPower, backLeftPower, backRightPower;

    private CRServo intake1;
    private CRServo intake2;


    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        intake1 = hardwareMap.get(CRServo.class, "servo0");
        intake2 = hardwareMap.get(CRServo.class, "servo1");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
    }

    public void start(){

    }

    public void loop(){

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);

        frontLeftPower  = (y + x + r) / denominator;
        frontRightPower = (y - x - r) / denominator;
        backLeftPower   = (y - x + r) / denominator;
        backRightPower  = (y + x - r) / denominator;

        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);


        if (gamepad1.right_bumper) {
            intake1.setPower(1);
            intake2.setPower(1);
        }

        if (gamepad1.left_bumper) {
            intake1.setPower(-1);
            intake2.setPower(-1);
        }

        if (gamepad1.a) {
            intake1.setPower(0);
            intake2.setPower(0);
        }








        telemetry.addData("LeftFront: ", leftFront.getCurrentPosition());
        telemetry.addData("LeftBack: ", leftBack.getCurrentPosition());
        telemetry.addData("RightFront: ", rightFront.getCurrentPosition());
        telemetry.addData("RightBack: ", rightBack.getCurrentPosition());

        telemetry.update();

    }

}
