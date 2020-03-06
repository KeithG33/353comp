
import anki_vector as av
from sys import argv

def main():
    # Modify the SN to match your robot’s SN
    ANKI_SERIAL = '00302fc4'
    ANKI_BEHAVIOR = av.connection.ControlPriorityLevel.OVERRIDE_BEHAVIORS_PRIORITY

    with av.Robot(serial=ANKI_SERIAL,
                  behavior_control_level=ANKI_BEHAVIOR) as robot:
        print("Say 'Hello World'...")
        robot.behavior.say_text("Ride or die")


if __name__ == "__main__":
	main()

