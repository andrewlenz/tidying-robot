#!/usr/bin/env python3

import asyncio
from bleak import BleakClient, BleakError
from ble_utils import parse_ble_args, handle_sigint
# args = parse_ble_args('Communicates with buckler display writer characteristic')
# timeout = args.timeout
# handle_sigint()

ANGLE_SERVICE_UUID = "32e69998-2b22-4db5-a914-43ce41986c70"
ANGLE_CHAR_UUID    = "32e61999-2b22-4db5-a914-43ce41986c70"
address = "C0:98:E5:49:00:05"
# address = addr.lower()

LAB11 = 0x02e0


class RobotController():
    def __init__(self):
        self.client = BleakClient(address)
        self.address = address
        self.commands = {"angle" : 0, "obstacle" : 7}

    # def send_angle(self, angle):
    #     self.commands[angle] = 

    # # def send_obstacle

    async def send_angle(self, angle):
        angle = str(angle)[:5]
        print(angle)
        if not self.client.is_connected():
            self.setup()
            print("BLE setup done before angle")
        try:
            await self.client.write_gatt_char(ANGLE_CHAR_UUID, bytes(angle, "utf-8"))
        except Exception as e:
            print(f"SEND ANGLE ERROR:\t{e}")

    async def setup(self):
        if not self.client.is_connected():
            try:
                await self.client.connect()
                # async with self.client as client:
                    # model_number = await client.read_gatt_char(DISPLAY_CHAR_UUID)
                print("Connected to device")
            # robot = RobotController(client)
                try:
                    print("Robot control enabled")
                except KeyboardInterrupt as e:
                    sys.exit(0)
                # finally: 
                #     await self.client.disconnect()
            except BleakError as e:
                print(f"not found:\t{e}")
        else:
            # print("Already connected")
            return     

# if __name__ == "__main__":
#     while True:
#         asyncio.run(main(addr))

# def run_main():
#     addr = "1600C8ED-2C0B-4FB3-82C8-6626050A8EB7"
#     address = addr.lower()
#     while True:
#         asyncio.run(main(address))
