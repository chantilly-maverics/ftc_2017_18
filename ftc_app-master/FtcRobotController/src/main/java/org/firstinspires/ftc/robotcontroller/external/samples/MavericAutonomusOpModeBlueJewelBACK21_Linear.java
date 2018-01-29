/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="ColorSensor", group="Sensor")
@Disabled
public class MavericAutonomusOpModeBlueJewelBACK21_Linear extends LinearOpMode {

    // Declare OpMode members.
    HardwarePushbot robot = new HardwarePushbot();
    private ElapsedTime limiter= new ElapsedTime();

    //public final static double COLORWRIST_MIN_RANGE = 0.0;
    //public final static double COLORWRIST_MAX_RANGE = 0.5;

    public Servo ColorArm = null;
    public Servo ColorWrist = null;

    public DcMotor left = null;
    public DcMotor right = null;
    public ColorSensor color;
    public final double LIMIT = 6;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;



    @Override
    public void runOpMode() {

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        boolean bLedOn = true;


        color = hardwareMap.colorSensor.get("ColorSensor");
        ColorArm = hardwareMap.servo.get("ColorArm");
        ColorWrist = hardwareMap.servo.get("ColorWrist");
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //telemetry.addData("Initialization", "Complete");
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);

        double rightCurrentPosition = right.getCurrentPosition();
        double leftCurrentPosition = left.getCurrentPosition();
        right.setPower(0.25);
        left.setPower(0.25);
        double turnspeed = 0.15;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        color.enableLed(bLedOn);


        //left.setDirection(DcMotor.Direction.FORWARD);
        //right.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {

            if (angles.firstAngle > 45) {
                left.setPower(turnspeed);
            } else {
                left.setPower(0);
            }
            telemetry.update();


            ColorArm.setPosition(0.1);
            sleep(2000);
            ColorWrist.setPosition(0.35);
            sleep(2000);
            ColorArm.setPosition(0.7);
            sleep(4000);


            if (color.blue() > color.red() && color.blue() > color.green()) {
                ColorWrist.setPosition(0.5);
                sleep(2000);
                ColorArm.setPosition(0.1);
                sleep(1500);
                ColorWrist.setPosition(0.85);
                sleep(500);
                ColorArm.setPosition(0);
                sleep(3000);

                left.setTargetPosition(4800);// moving robot forward into parking zone
                right.setTargetPosition(4800);
                left.setPower(0.65);
                right.setPower(0.65);
                sleep(30000000);


            } else if (color.red() > color.blue() && color.red() > color.green()) {
                ColorWrist.setPosition(0.10);// Blue servo is activated
                sleep(1000);
                ColorArm.setPosition(0.1);
                sleep(1500);
                ColorWrist.setPosition(0.85);
                sleep(500);
                ColorArm.setPosition(0);
                sleep(3000);
                left.setTargetPosition(4800);
                right.setTargetPosition(4800);
                left.setPower(0.65);
                right.setPower(0.65);
                sleep(30000000);

            } else {
                ColorWrist.setPosition(0.85);
                sleep(5000);
                ColorArm.setPosition(0);
                sleep(1000);
                left.setTargetPosition(4800);
                right.setTargetPosition(4800);
                left.setPower(0.5);
                right.setPower(0.65);
                sleep(30000000);


            }


        }
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();


            }
        });

    }

            /*telemetry.addLine()
                    .addData("status", new Func<String>() {
                        @Override public String value() {
                            return imu.getSystemStatus().toShortString();
                        }
                    })
                    .addData("calib", new Func<String>() {
                        @Override public String value() {
                            return imu.getCalibrationStatus().toString();
                        }
                    });

            telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.thirdAngle);
                        }
                    });

            telemetry.addLine()
                    .addData("grvty", new Func<String>() {
                        @Override public String value() {
                            return gravity.toString();
                        }
                    })
                    .addData("mag", new Func<String>() {
                        @Override public String value() {
                            return String.format(Locale.getDefault(), "%.3f",
                                    Math.sqrt(gravity.xAccel*gravity.xAccel
                                            + gravity.yAccel*gravity.yAccel
                                            + gravity.zAccel*gravity.zAccel));
                        }
                    });
        String formatAngle(AngleUnit double angle) {
            return formatAngle(AngleUnit.DEGREES.fromUnit(angles.toAngleUnit());
        }

        String formatDegrees(double degrees);{
            return String.formatDegrees(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        }



        right.setPower(0);
            left.setPower(0);

            //right.setPower(0);
            //left.setPower(0);




            }
        }

    */

}


