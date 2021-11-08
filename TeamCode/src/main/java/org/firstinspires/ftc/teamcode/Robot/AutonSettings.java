package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.parkTypes.*;

public class AutonSettings {

    Gamepad gamepad;
    parkTypes parkType = REGULAR;
    Telemetry telemetry;
    double minimumParkDelay;
    double maximumParkDelay;

    double chosenParkDelay;

    public AutonSettings(Gamepad gamepad, Telemetry telemetry, int minumumParkDelay, int maximumParkDelay) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.minimumParkDelay = minumumParkDelay;
        this.maximumParkDelay = maximumParkDelay;
        chosenParkDelay = minimumParkDelay;
    }

    public void chooseSettings() throws InterruptedException {
        boolean currentUp = false,  prevUp = false, currentDown = false, prevDown = false, currentB = false, prevB = false;

        while(!gamepad.a) {
            currentUp = gamepad.dpad_up;
            currentDown = gamepad.dpad_down;
            currentB = gamepad.b;

            if(currentUp && currentUp != prevUp) {
                if(chosenParkDelay <= maximumParkDelay + 1) {
                    chosenParkDelay++;
                }
            }else if (currentDown && currentDown != prevDown) {
                if(chosenParkDelay >= minimumParkDelay - 1) {
                    chosenParkDelay--;
                }
            }

            if(currentB && currentB != prevB) {
                if(parkType == REGULAR) {
                    parkType = OFFSET;
                } else if(parkType == OFFSET) {
                    parkType = SHIPPING_AREA;
                } else if(parkType == SHIPPING_AREA) {
                    parkType = REGULAR;
                }
            }

            telemetry.addData("Current Settings", "");
            telemetry.addData("Park Delay (dpad up and down) ", chosenParkDelay);
            telemetry.addData("Park Type (b)", parkType);
            telemetry.addData("Lock in settings", "a");
            telemetry.update();

            prevUp = currentUp;
            prevDown = currentDown;
            prevB = currentB;
            Thread.sleep(20);
        }

        telemetry.addData("SETTINGS SELECTED", "");
        telemetry.addData("Park Delay", chosenParkDelay);
        telemetry.addData("Park Type", parkType);
        telemetry.update();
    }

    public enum parkTypes {
        REGULAR, OFFSET, SHIPPING_AREA
    }

    public parkTypes getParkType() {
        return parkType;
    }

    public void setParkType(parkTypes parkType) {
        this.parkType = parkType;
    }

    public double getChosenParkDelay() {
        return chosenParkDelay*1000;
    }

    public void setChosenParkDelay(double chosenParkDelay) {
        this.chosenParkDelay = chosenParkDelay / 1000.0;
    }
}
