#!/usr/bin/env python3

import asyncio
from bleak import BleakClient, BleakError
from ble_utils import parse_ble_args, handle_sigint
# args = parse_ble_args('Communicates with buckler display writer characteristic')
# timeout = args.timeout
# handle_sigint()

ANGLE_SERVICE_UUID = "32e69998-2b22-4db5-a914-43ce41986c65"

# angle to turn
ANGLE_CHAR_UUID = "32e61999-2b22-4db5-a914-43ce41986c65"

# true to grab, else false
# GRAB_CHAR_UUID = "32e62999-2b22-4db5-a914-43ce41986c65"

# true to deposit, else false
READY_CHAR_UUID = "32e63999-2b22-4db5-a914-43ce41986c65"

# will read true if robot has picked up object
PICKED_CHAR_UUID = "32e64999-2b22-4db5-a914-43ce41986c65"

# true if robot should stop driving
ARRIVED_CHAR_UUID = "32e65999-2b22-4db5-a914-43ce41986c65"

# true if robot should start driving cautiously
DRIVE_CAUTIOUS_UUID = "32e66999-2b22-4db5-a914-43ce41986c65"

address = "C0:98:E5:49:20:80"
# address = addr.lower()

LAB11 = 0x02e0

# 12 bytes
# bytes 0-4: angle
# byte  5  : grab
# byte  6  : deposit
# byte  7  : pick
# byte  8  : arrived
# byte  9  : drive cautious


class RobotController():
    def __init__(self):
        self.client = BleakClient(address, use_cached=False)
        self.address = address
        self.commands = {"angle" : 0, "obstacle" : 7}
        self.new = False
        self.angle = 0
        self.wait = 0
        # self.grab = 0
        self.ready = 0
        self.pick = 0
        self.arrived = 0
        self.drive_cautious = 0

    async def edit_angle(self, angle):
        self.angle = angle
        self.new = True

    async def edit_arrived(self):
        self.arrived = 1
        # print("edit", self.arrived)

    async def edit_ready(self):
        self.ready = 1

    async def edit_drive_cautious(self):
        self.drive_cautious = 1

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

    async def send(self):
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
                # i = await self.client.get_services()
                # print([i.characteristics[c].uuid for c in i.characteristics])
                # await self.check_angle()
                # i = await self.client.get_services()
                # print(i.characteristics)
                # print("angle: ", self.angle)
                # print("self.new: ", self.new)
                # print(self.grab, self.deposit, self.pick, self.arrived)
                # if self.angle != 0 and self.new:
                #     self.send_val = ((self.send_val >> 32) << 32) | int(self.angle*100)
                # if self.grab:
                #     self.send_ops |= (1 << 0)
                # if self.deposit:
                #     self.send_ops |= (1 << 1)
                # if self.pick:
                #     self.send_ops |= (1 << 2)
                # if self.arrived:
                #     self.send_ops |= (1 << 3)
                # print("sent info")
                # print(hex(self.send_ops), hex(self.send_val))
                angle = str(self.angle)[:5]
                await self.client.write_gatt_char(ANGLE_CHAR_UUID, bytes(str(angle), "utf-8"))
                await self.client.write_gatt_char(ARRIVED_CHAR_UUID, bytes(str(self.arrived), "utf-8"))
                await self.client.write_gatt_char(READY_CHAR_UUID, bytes(str(self.ready), "utf-8"))
                await self.client.write_gatt_char(DRIVE_CAUTIOUS_CHAR_UUID, bytes(str(self.drive_cautious), "utf-8"))
                self.new = False
                # self.grab = 0
                self.deposit = 0
                # self.ready = 0
                self.arrived = 0
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
            i = await self.client.get_services()
            print(i.characteristics)
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
