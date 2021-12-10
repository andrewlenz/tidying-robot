#!/usr/bin/env python3

import asyncio
from bleak import BleakClient, BleakError
from ble_utils import parse_ble_args, handle_sigint
# args = parse_ble_args('Communicates with buckler display writer characteristic')
# timeout = args.timeout
# handle_sigint()

ANGLE_SERVICE_UUID = "32e69998-2b22-4db5-a914-43ce41986c65"
ANGLE_CHAR_UUID    = "32e61999-2b22-4db5-a914-43ce41986c65"
address = "C0:98:E5:49:20:00"
# address = addr.lower()

LAB11 = 0x02e0



class RobotController():
    def __init__(self):
        self.client = BleakClient(address)
        self.address = address
        self.commands = {"angle" : 0, "obstacle" : 7}
        self.new = False
        self.angle = 0

    def edit_angle(self, angle):
        self.angle = angle
        self.new = True

    async def disconnect(self):
        if self.client.is_connected:
            print("disconnecting at start")
            await self.client.disconnect()
    # def send_angle(self, angle):
    #     self.commands[angle] = 

    # # def send_obstacle

    async def check_angle(self):
        while robot1_angle == 0:
            continue
        return True

    async def send_angle(self):
        while True:
        # self.client = BleakClient(address)
        # self.angle = angle
            # print("SEND ANGLE")
            # print(angle)
            # while True: 
            if not self.client.is_connected:
                await self.setup()
                print("BLE setup done before angle")
            try:
                # await self.check_angle()
                # i = await self.client.get_services()
                # print(i.characteristics)
                # print("angle: ", self.angle)
                # print("self.new: ", self.new)
                if self.angle != 0 and self.new == True:
                    angle = str(self.angle)[:5]
                    await self.client.write_gatt_char(ANGLE_CHAR_UUID, bytes(angle, "utf-8"))
                    print("sent angle")
                    self.new = False
                await asyncio.sleep(0)
            except BleakError as e:
                print(f"BLEAK:\t{e}")
            except Exception as e:
                print(f"SEND ANGLE ERROR:\t{e}")

    async def setup(self):
        # self.client = BleakClient(address)
        # print(self.client.is_connected)
        if not self.client.is_connected:
            print("doing ble setup")
            try:
                await self.client.connect()
                # async with self.client as client:
                    # model_number = await client.read_gatt_char(DISPLAY_CHAR_UUID)
                print("Connected to device")
            # robot = RobotController(client)
                # try:
            #     print("Robot control enabled")
            except KeyboardInterrupt as e:
                sys.exit(0)
                # finally: 
                #     await self.client.disconnect()
            except BleakError as e:
                print(f"not found:\t{e}")
                self.setup()
        else:
            # print("Was already connected, here are the services")
            # i = await self.client.get_services()
            # print(i.characteristics)
            return     
        return

# if __name__ == "__main__":
#     while True:
#         asyncio.run(main(addr))

# def run_main():
#     addr = "1600C8ED-2C0B-4FB3-82C8-6626050A8EB7"
#     address = addr.lower()
#     while True:
#         asyncio.run(main(address))
