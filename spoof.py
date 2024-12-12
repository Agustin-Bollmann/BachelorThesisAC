import can
import time

def send_can_message():
    # Initialize the virtual CAN bus (vcan0)
    bus = can.interface.Bus(channel='vcan0', bustype='socketcan')
    
    # Define the single CAN message
    message = can.Message(
        arbitration_id=0x110,
        data=[
        0x7B, # Brake 123
        0x53, #Steering 83
        0x6A],  #Throttle 106
        is_extended_id=False
    )

    print("Sending CAN message continuously. Press Ctrl+C to stop.")

    try:
        while True:
            bus.send(message)  # Send the CAN message
            data_in_decimal = list(message.data)
            print(f"Sent CAN message: ID={message.arbitration_id}, Data={data_in_decimal}")
            time.sleep(0.1)  # Optional: Adjust to control sending speed (100 ms delay)
    except KeyboardInterrupt:
        print("\nProgram interrupted. Stopping message sending.")
    except can.CanError as e:
        print(f"CAN Error occurred: {e}")
    finally:
        bus.shutdown()  # Clean up the CAN bus connection

if __name__ == "__main__":
    send_can_message()
