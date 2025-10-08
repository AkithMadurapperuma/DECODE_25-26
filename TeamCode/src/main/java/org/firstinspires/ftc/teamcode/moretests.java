package org.firstinspires.ftc.teamcode;

import com.google.gson.JsonElement;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.LED;



import java.io.IOException;
import java.net.MalformedURLException;
import java.net.ProtocolException;
import java.util.ArrayList;
import java.util.List;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import com.google.gson.JsonSyntaxException;



@TeleOp(name = "moretests")
public class moretests extends LinearOpMode {

    private Limelight3A limelight;

    private ColorSensor colorSensor;
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    double motor0power;
    double motor1power;
    double motor2power;
    double motor3power;
    private LED led;
    private CRServo servo;

    double Kp = 0.02;
    double Ki = 0.001;
    double Kd = 0.002;

    double integralSum = 0;
    double lastError = 0;
    double P, I, D;





    private final String limelightIP = "172.29.0.1";

    private volatile int fiducialID = -1;

    private boolean gpp = false;
    private boolean pgp = false;
    private boolean ppg = false;



        @Override
        public void runOpMode() throws InterruptedException {

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            colorSensor = hardwareMap.get(ColorSensor.class, "color");
            motor0 = hardwareMap.get(DcMotor.class, "motor0");
            motor1 = hardwareMap.get(DcMotor.class, "motor1");
            motor2 = hardwareMap.get(DcMotor.class, "motor2");
            motor3 = hardwareMap.get(DcMotor.class, "motor3");
            led = hardwareMap.get(LED.class, "led");
            servo = hardwareMap.get(CRServo.class, "servo");


            motor0.setDirection(DcMotor.Direction.REVERSE);
            motor1.setDirection(DcMotor.Direction.FORWARD);
            motor2.setDirection(DcMotor.Direction.REVERSE);
            motor3.setDirection(DcMotor.Direction.FORWARD);


            limelight.start();

            // Start a thread to poll Limelight HTTP continuously
            Thread limelightThread = new Thread(() -> {
                while (!Thread.currentThread().isInterrupted() && !isStopRequested()) {
                    fiducialID = getFiducialID(limelightIP);
                    try {
                        Thread.sleep(100); // 100 ms polling interval
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            });
            limelightThread.start();

            waitForStart();

            while (opModeIsActive()) {

                limelight.start();




                    if (fiducialID != -1) {
                    telemetry.addData("Fiducial ID (tid)", fiducialID);
                } else {
                    telemetry.addData("Fiducial ID (tid)", "No tag detected");
                }
                telemetry.update();

                LLResult result = limelight.getLatestResult();
                Pose3D botpose = result.getBotpose();




                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());


                if(fiducialID == 21){
                    gpp = true;
                }
                else if(fiducialID == 22){
                    pgp = true;
                }
                else if(fiducialID == 23){
                    ppg = true;
                }


                double red = colorSensor.red();
                double blue = colorSensor.blue();
                double green = colorSensor.green();
                //    double alpha = colorSensor.alpha();

                boolean greenball = false;
                boolean purpleball = false;

                double r = red / 255.0;
                double g = green / 255.0;
                double b = blue / 255.0;

                // h, s, v = hue, saturation, value
                double cmax = Math.max(r, Math.max(g, b));
                double cmin = Math.min(r, Math.min(g, b));
                double diff = cmax - cmin;
                double h = -1, s = -1;

                if (cmax == cmin)
                    h = 0;

                else if (cmax == r)
                    h = (60 * ((g - b) / diff) + 360) % 360;

                else if (cmax == g)
                    h = (60 * ((b - r) / diff) + 120) % 360;

                else if (cmax == b)
                    h = (60 * ((r - g) / diff) + 240) % 360;

                if (cmax == 0)
                    s = 0;
                else
                    s = (diff / cmax) * 100;

                double v = cmax * 100;
                //telemetry.addData("Hue", h);
                //telemetry.addData("Saturation", s);
                //telemetry.addData("Value", v);


                if (h < 360 && h > 310) {
                    purpleball = true;
                    led.enable(true);
                }

                if (h < 140 && h > 105) {
                    greenball = true;
                    led.enable(true);
                }



                telemetry.addData("Greenball", greenball);
                telemetry.addData("Purpleball", purpleball);

                double x = gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y;
                double r1 = gamepad1.right_stick_x;
                double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r1), 1);


                if(gamepad1.x) {
                    if (fiducialID == 24 || fiducialID == 22 ) {
                        double tx = result.getTx();
                        double error = -tx;

                        P = Kp * error;

                        integralSum += error;
                        I = Ki * integralSum;

                        D = Kd * (error - lastError);

                        double correction = P + I + D;
                        lastError = error;


                        double turnPower = correction;



                        // Example for a tank drive
                        motor0.setPower(-turnPower);
                        motor2.setPower(-turnPower);
                        motor1.setPower(turnPower);
                        motor3.setPower(turnPower);

                        // Add telemetry for debugging
                        telemetry.addData("Error", error);
                        telemetry.addData("Correction", correction);
                        telemetry.update();
                    } else {

                    }

                }else{
                    motor0power = (y + x + r1) / denominator;
                    motor1power = (y - x - r1) / denominator;
                    motor2power = (y - x + r1) / denominator;
                    motor3power = (y + x - r1) / denominator;
                }


                motor0.setPower(motor0power);
                motor1.setPower(motor1power);
                motor2.setPower(motor2power);
                motor3.setPower(motor3power);

            }

            limelightThread.interrupt();
        }

        // Helper method to get the fiducial ID from Limelight JSON
        private int getFiducialID(String ip) {
            try {
                URL url = new URL("http://" + ip + ":5807/results");
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setRequestMethod("GET");
                conn.setConnectTimeout(200);
                conn.setReadTimeout(200);

                StringBuilder sb = new StringBuilder();
                try (BufferedReader in = new BufferedReader(new InputStreamReader(conn.getInputStream()))) {
                    String line;
                    while ((line = in.readLine()) != null) {
                        sb.append(line);
                    }
                }

                JsonParser parser = new JsonParser();
                JsonElement rootElem = parser.parse(sb.toString());
                if (!rootElem.isJsonObject()) return -1;
                JsonObject root = rootElem.getAsJsonObject();

                // Check "Results" array
                if (root.has("Fiducial") && root.get("Fiducial").isJsonArray()) {
                    JsonArray results = root.getAsJsonArray("Fiducial");
                    if (results.size() > 0 && results.get(0).isJsonObject()) {
                        JsonObject first = results.get(0).getAsJsonObject();
                        if (first.has("fID") && !first.get("fID").isJsonNull()) {
                            return first.get("fID").getAsInt();
                        }
                    }
                }



                return -1; // no tag detected
            } catch (Exception e) {
                e.printStackTrace();
                return -1;
            }
        }
    }