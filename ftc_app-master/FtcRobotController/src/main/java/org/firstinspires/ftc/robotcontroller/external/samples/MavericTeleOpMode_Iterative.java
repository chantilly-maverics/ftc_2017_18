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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *n hj
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 * comment by Akhil
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
@Disabled

public class MavericTeleOpMode_Iterative extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor middle = null;
    private DcMotor relicLinear = null;
    //private DcMotor arm = null;
    //private CRServo wrist;
    private DcMotor linearArm = null;
    private Servo claw;
    private Servo claw2;
    private Servo relicWrist = null;
    private CRServo relicClaw = null;



    //public final static double WRIST_HOME = 0.5;
    //public final static double WRIST_MIN_RANGE = 0.0;
    //public final static double WRIST_MAX_RANGE = 1;

      //public final static double NECK_MIN_RANGE = 0.5;
      //public final static double NECK_MAX_RANGE = 1.0;
      //double wristPosition = 0.5;
      //final double NECK_SPEED = 1.0;

    /*double linearArmPosition = 0.5;
    public final static double ARM_HOME = 0.49;
    public final static double ARM_MIN_RANGE = 0.45;
    public final static double ARM_MAX_RANGE = 0.55;
    double linearPosition = ARM_HOME;
    final double ARM_SPEED = 0.8;

    //double wristPosition = WRIST_HOME;
    //final double WRIST_SPEED = 0.8;










    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        middle = hardwareMap.get(DcMotor.class, "middle");
        relicLinear = hardwareMap.get(DcMotor.class, "relicLinear");
        linearArm = hardwareMap.get(DcMotor.class, "LinearArm");
        claw = hardwareMap.get(Servo.class, "claw");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        relicWrist = hardwareMap.get(Servo.class, "relicWrist");
        relicClaw = hardwareMap.get(CRServo.class, "relicClaw");
        //arm = hardwareMap.get(DcMotor.class, "arm");
        //wrist = hardwareMap.get(CRServo.class, "wrist");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);
        linearArm.setDirection(DcMotor.Direction.REVERSE);
        //linearArm.setDirection(DcMotorSimple.Direction.REVERSE);
        //wrist.setDirection(DcMotorSimple.Direction.REVERSE);
        //linearArm.setDirection(DcMotor.Direction.FORWARD);
        relicLinear.setDirection(DcMotor.Direction.REVERSE);

        claw.setPosition(0.4);
        claw2.setPosition(0.6);

        //linearArm.setPosition(.49);



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double linearArmPower;
        double relicLinearPower;



        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.


        double drive1 = -gamepad1.left_stick_y;
        double drive = -gamepad1.right_stick_y;
        double turn1 = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double lift = gamepad2.left_stick_x;
        double lift2 = -gamepad2.left_stick_y;



        double push = gamepad2.right_stick_x;
        double push2 = -gamepad2.right_stick_y;



        linearArmPower = Range.clip(lift2 - lift,-0.8,0.8);
        leftPower = Range.clip(drive1 + turn1, -1, 1);
        rightPower = Range.clip(drive - turn, -1, 1);
        relicLinearPower = Range.clip(push2 - push, -1,1);


        //double linearPosition = -gamepad2.right_stick_y;

        //linearArmPower = Range.clip(linearPosition,ARM_MIN_RANGE,ARM_MAX_RANGE);
        //double push = gamepad2.left_stick_x;
        //double push2 = gamepad2.left_stick_y;

        //armPower = Range.clip(lift, -0.2, 0.6);


        //wristPosition = Range.clip(wristPosition,WRIST_MIN_RANGE,WRIST_MAX_RANGE);
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        if (gamepad1.left_bumper) {
            middle.setPower(1);// holding x full speed ahead
        } else if (gamepad1.right_bumper) {
            middle.setPower(-1);// holding y full reverse
        } else {
            middle.setPower(0);// Stop
        }


        //if (gamepad2.right_bumper ) {
            //linearArmPosition = 0.5;
           // linearPosition = linearArm.getPosition() + 0.1;
            //linearPosition = Range.clip(linearPosition, 0.2, 0.3);
            //linearArm.setPosition(linearPosition);


        /*if (gamepad2.y) {
            wrist.setPosition(1);


        } else if (gamepad2.a) {
            wrist.setPosition(0);
        } */

        /*if (gamepad2.y) {
            neckPosition += NECK_SPEED;
        }else if (gamepad2.a) {
            neckPosition = 0;
        } */



        if (gamepad2.b) {
            claw.setPosition(.4);
            claw2.setPosition(.6);

        } else if (gamepad2.x) {
            claw.setPosition(.7);
            claw2.setPosition(.3);
        }



        if (gamepad1.y) {
            relicClaw.setPower(0.8);
        }else if (gamepad1.a) {
            relicClaw.setPower(-0.7);
        }


        /*if (gamepad2.y)
        {
            linearArm.setPower(0.02);

        }
        else if (gamepad2.a)
        {
            linearArm.setPower(-0.2);
        }       */

        if (gamepad2.right_bumper) {
            //if (relicWrist.getPosition() <= 1.00)

            relicWrist.setPosition(0);

        }else if (gamepad2.left_bumper) {
            //if (relicWrist.getPosition() >= 0.00)
            relicWrist.setPosition(0.6);
        }





                //x and y buttons controlling middle motor

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        left.setPower(leftPower);
        right.setPower(rightPower);
        //linearArm.setPower(linearArmPower);
        relicLinear.setPower(relicLinearPower);
        linearArm.setPower(linearArmPower);

        //arm.setPower(armPower);
        //wrist.setPower(wristPower);







        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower, linearArmPower);
    }

    /*c
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
