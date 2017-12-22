/*
FTC Team 4991 GearFreaks
Author:  Joe Walton
Date: 29 Oct 2017
Modified by:
Modified Date:
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="relicrescue_tele")
public class relicrescue_tele extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor driveLeftfront;
    DcMotor driveRightfront;

    DcMotor driveLeftback;
    DcMotor driveRightback;

    DcMotor linearLift;

    Servo claw;

    double CLAW_OPEN = 0.90;
    double CLAW_CLOSE = 0.10;

    boolean buttonA;
    boolean buttonB;
    boolean buttonX;
    boolean buttonY;

    boolean lbumper;
    boolean rbumper;
    float ltrigger;
    float rtrigger;


    float stage_up;
    float stage_down;

    float stage_0;
    int liftPosition=0;
    int target = 1350;
    int GOOD=0;
    int small = 100;
    // target is negative so + goes up and - goes down


    //DcMotorController.DeviceMode DevMode;
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        driveRightfront = hardwareMap.dcMotor.get("rightfront");
        driveRightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveLeftfront = hardwareMap.dcMotor.get("leftfront");
        driveLeftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveRightback = hardwareMap.dcMotor.get("rightback");
        driveRightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveLeftback = hardwareMap.dcMotor.get("leftback");
        driveLeftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // int ANDYMARK_TICKS_PER_REV = 1120;
        // int TETRIX_TICKS_PER_REV = 1440;
        linearLift = hardwareMap.dcMotor.get("linearlift");
        linearLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveRightfront.setDirection(DcMotor.Direction.REVERSE);
        driveRightback.setDirection(DcMotor.Direction.REVERSE);
        linearLift.setDirection(DcMotor.Direction.REVERSE);

        claw = hardwareMap.servo.get("claw");

        claw.setPosition(CLAW_OPEN);

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive() ) {
            //getJoystickSettings(joystick);

            //left joystick controls the robot for:
            //     forward
            //     reverse
            //     straf left (sideways)
            //     straf right (sideways)
            //     and all diagonals
            float Yvalue1 = gamepad1.left_stick_y;
            float Xvalue1 = gamepad1.left_stick_x;

            //right joystick controls the turning!!
            float Yvalue2 = -gamepad1.right_stick_y;
            float Xvalue2 = -gamepad1.right_stick_x;

            float LF = (Yvalue1 - Xvalue1);
            float RF = (Yvalue1 + Xvalue1);
            float LB = (Yvalue1 + Xvalue1);
            float RB = (Yvalue1 - Xvalue1);

            LF = Range.clip(LF, -1, 1);
            RF = Range.clip(RF, -1, 1);
            LB = Range.clip(LB, -1, 1);
            RB = Range.clip(RB, -1, 1);

            // only use if turning robot --- MAY HAVE TO REVERSE THE VALYES
            if(Xvalue2 != 0){
                RF -= Xvalue2;
                RB -= Xvalue2;
                LF += Xvalue2;
                LB += Xvalue2;
            }

            driveLeftfront.setPower(LF);
            driveLeftback.setPower(LB);
            driveRightfront.setPower(RF);
            driveRightback.setPower(RB);

//            float BYvalue1 = -gamepad2.left_stick_y;
//            float BXvalue1 = -gamepad2.left_stick_x;
//            float linear_power = (BYvalue1 + BXvalue1);
//
//            linear_power = Range.clip(linear_power, -1, 1);
//            //linearLift.setPower(linear_power);

            // get current button values
            lbumper = gamepad2.left_bumper;
            rbumper = gamepad2.right_bumper;
            ltrigger = gamepad2.left_trigger;
            rtrigger = gamepad2.right_trigger;
            buttonA = gamepad2.a;
            buttonB = gamepad2.b;
            buttonX = gamepad2.x;
            buttonY = gamepad2.y;


            // target is negative so + goes up and - goes down
            // need to fine tune the target distance from 3000 ticks to ???
            if(buttonA && GOOD == 0) {
                GOOD=1;
                liftPosition = (linearLift.getCurrentPosition());
                liftPosition -= target;
                if(liftPosition < 0){
                    liftPosition=0;
                }
                linearLift.setTargetPosition(liftPosition);
                linearLift.setPower(1.00);

                telemetry.addData("GOING TO A", liftPosition);    //
                telemetry.update();
            }

            if(buttonB && GOOD == 0) {
                GOOD=1;
                liftPosition = (linearLift.getCurrentPosition());
                linearLift.setPower(1.00);
                liftPosition += target;
                if(liftPosition > 4050){
                    liftPosition=4050;
                }
                linearLift.setTargetPosition(liftPosition);

                telemetry.addData("GOING TO B", liftPosition);    //
                telemetry.update();
            }
            if(Math.abs(linearLift.getCurrentPosition() - linearLift.getTargetPosition()) <= 20){
                GOOD = 0;
            }
            if(buttonX) {
                linearLift.setPower(0.00);
            }
            else if(buttonY) {
                linearLift.setPower(0.00);
            }


            if(lbumper) {   // BUMPER IS BOOLEAN: either pressed or not pressed
                // OPEN claw
                claw.setPosition(CLAW_CLOSE);
            }
            else if(rbumper) { //Trigger is FLOAT, measure between 0.0 abd 1.0
                // CLOSE claw
                claw.setPosition(CLAW_OPEN);
            }


            if(ltrigger > 0.0) {   // BUMPER IS BOOLEAN: either pressed or not pressed
                //move the linearLift up
                GOOD=1;
                liftPosition = (linearLift.getCurrentPosition());
                linearLift.setPower(1.00);
                liftPosition -= small;
                //if(liftPosition < 0){
                //    liftPosition=0;
                //}

                linearLift.setTargetPosition(liftPosition);

                telemetry.addData("GOING TO ADJUST POS LT", liftPosition);    //
                telemetry.update();
            }
            else if(rtrigger > 0.0) { //Trigger is FLOAT, measure between 0.0 abd 1.0
                // move the linearLift down
                GOOD=1;
                liftPosition = (linearLift.getCurrentPosition());
                liftPosition += small;
                //if(liftPosition > 4050){
                //    liftPosition=4050;
                //}
                linearLift.setTargetPosition(liftPosition);
                linearLift.setPower(1.00);

                telemetry.addData("GOING TO ADJUST POS RT", liftPosition);    //
                telemetry.update();
            }
//             else {
//                linearLift.setPower(0.0);
//            }
            telemetry.addData("CurrPos", linearLift.getCurrentPosition());
            telemetry.addData("Good", GOOD);
            telemetry.update();
        }
        // end of OpMode = Game over/stop pushed

        driveLeftfront.setPower(0.00);
        driveRightfront.setPower(0.00);
        driveLeftback.setPower(0.00);
        driveRightback.setPower(0.00);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        //sleep(1000);
        idle();
    }

}
