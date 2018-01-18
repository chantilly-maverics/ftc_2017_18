package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

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

public class MavericAutonomusOpModeBlueJewelBACK_Linear extends LinearCommon {
    private void moveForward() {
        sleep(1000);
        leftDc.setTargetPosition(4800);
        rightDC.setTargetPosition(4800);
        leftDc.setPower(0.5);
        rightDC.setPower(0.65);
        sleep(30000000);
    }

    private void positionInbetweenJewel() {
        ColorArmServo.setPosition(0.1);
        sleep(2000);
        ColorWristServo.setPosition(0.35);
        sleep(2000);
        ColorArmServo.setPosition(0.7);
        sleep(4000);
    }
}


