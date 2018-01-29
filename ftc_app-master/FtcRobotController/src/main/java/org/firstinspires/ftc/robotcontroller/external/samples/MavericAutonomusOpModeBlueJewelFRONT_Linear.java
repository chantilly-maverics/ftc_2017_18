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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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
public class MavericAutonomusOpModeBlueJewelFRONT_Linear extends LinearOpMode {

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
    public Servo claw;
    public Servo claw2;
    public DcMotor linearArm = null;
    public final double LIMIT = 6;


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
        left= hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        claw = hardwareMap.get(Servo.class,"claw");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        linearArm = hardwareMap.get(DcMotor.class, "LinearArm");


        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        //telemetry.addData("Initialization", "Complete");
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.REVERSE);
       
        color.enableLed(bLedOn);


        //left.setDirection(DcMotor.Direction.FORWARD);
        //right.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {


            ColorArm.setPosition(0.1);
            sleep(2000);
            ColorWrist.setPosition(0.4);
            sleep(2000);
            ColorArm.setPosition(0.9);
            sleep(4000);


            if (color.blue() > color.red() && color.blue() > color.green()) {
                ColorWrist.setPosition(0.2);
                sleep(2000);
                ColorArm.setPosition(0.1);
                sleep(1500);
                ColorWrist.setPosition(0.85);
                sleep(500);
                ColorArm.setPosition(0);
                sleep(3000000);
                /*claw.setPosition(0.7);
                claw2.setPosition(0.3);
                sleep(1000);
                linearArm.setTargetPosition(600);
                sleep(1000);
                left.setTargetPosition(4800);// moving robot forward into parking zone
                right.setTargetPosition(4800);
                left.setPower(1);
                right.setPower(1);
                sleep(30000000); */



            } else if (color.red() > color.blue() && color.red() > color.green()) {
                ColorWrist.setPosition(0.45);// Blue servo is activated
                sleep(1000);
                ColorArm.setPosition(0.1);
                sleep(1500);
                ColorWrist.setPosition(0.85);
                sleep(500);
                ColorArm.setPosition(0);
                sleep(3000000);
                /*claw.setPosition(0.7);
                claw2.setPosition(0.3);
                sleep(1000);
                linearArm.setTargetPosition(600);
                sleep(1000);
                left.setTargetPosition(4800);
                right.setTargetPosition(4800);
                left.setPower(1);
                right.setPower(1);
                sleep(30000000); */

            } else {
                ColorArm.setPosition(0.1);
                sleep(1000);
                ColorWrist.setPosition(0.85);
                sleep(1000);
                ColorArm.setPosition(0);
                /*claw.setPosition(0.7);
                claw2.setPosition(0.3);
                sleep(1000);
                linearArm.setTargetPosition(600);
                sleep(1000);
                left.setTargetPosition(4800);
                right.setTargetPosition(4800);
                left.setPower(0.65);
                right.setPower(0.65);
                sleep(30000000); */


            }


            }

            linearArm.setTargetPosition(0);
            right.setPower(0);
            left.setPower(0);

            //right.setPower(0);
            //left.setPower(0);

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });



            }
        }



