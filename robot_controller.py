#!/usr/bin/env python3

import asyncio
from bleak import BleakClient, BleakError
from ble_utils import parse_ble_args, handle_sigint

ANGLE_SERVICE_UUID = "32e69998-2b22-4db5-a914-43ce41986c65"
# angle to turn
ANGLE_CHAR_UUID = "32e61999-2b22-4db5-a914-43ce41986c65"
# true to deposit, else false
READY_CHAR_UUID = "32e63999-2b22-4db5-a914-43ce41986c65"
# will read true if robot has picked up object
PICKED_CHAR_UUID = "32e64999-2b22-4db5-a914-43ce41986c65"
# true if robot should stop driving
ARRIVED_CHAR_UUID = "32e65999-2b22-4db5-a914-43ce41986c65"
# true if robot should start driving cautiously
DRIVE_CAUTIOUS_CHAR_UUID = "32e66999-2b22-4db5-a914-43ce41986c65"


address2 = "C0:98:E5:49:30:01"
address1 = "C0:98:E5:49:30:02"

LAB11 = 0x02e0

class RobotController():
    def __init__(self, address):
        self.address = address # robot address
        self.client = BleakClient(address, use_cached=False)
        self.angle = 0 # robot's turning angle
        self.ready = 0 # robot is ready to go to next target
        self.pick = 0 # robot is holding an object, UPDATED BY ROBOT ONLY
        self.arrived = 0 # at target
        self.drive_cautious = 0 # should drive cautiously
        self.depositing = 0 # on route to drop off
        self.id = -1
        self.target = -1
        self.dist = -1
        self.orient = True
        self.counter = 0
        self.finished = 0
        # self.wait = 0 # robot is waiting UPDATED BY ROBOT ONLY

    async def disconnect(self):
        if self.client.is_connected:
            await self.client.disconnect()

    async def check_angle(self):
        while self.angle == 0:
            continue
        return True

    async def send(self):
        # print("finished")
        while True and not self.finished:
            if not self.client.is_connected:
                print("Client not connected, attempting connect")
                await self.setup()
                print("BLE setup done before angle")
            try:
                await asyncio.wait_for(self.client.write_gatt_char(ANGLE_CHAR_UUID, bytes(str(self.angle)[:6], "utf-8")), timeout = 5)
                await asyncio.wait_for(self.client.write_gatt_char(ARRIVED_CHAR_UUID, bytes(str(self.arrived), 'utf-8')), timeout = 5)
                await asyncio.wait_for(self.client.write_gatt_char(DRIVE_CAUTIOUS_CHAR_UUID, bytes(str(self.drive_cautious), 'utf-8')), timeout = 5)
                await asyncio.wait_for(self.client.write_gatt_char(READY_CHAR_UUID, bytes(str(self.ready), 'utf-8')), timeout = 5)
                temp_pick = await asyncio.wait_for(self.client.read_gatt_char(PICKED_CHAR_UUID), timeout = 5)
                self.pick = 0 if temp_pick.decode('utf-8')[0] == "0" else 1 #int(temp_pick.decode('utf-8'))
                formatted_angle = "{:.2f}".format(self.angle)
                print(f"ROBOT {self.id}, angle: {formatted_angle}, arrived: {self.arrived}, ready: {self.ready}, cautious: {self.drive_cautious}, depositing: {self.depositing}, pick: {self.pick}")
                self.counter = 0
            except asyncio.TimeoutError:
                print("Had a time out error")
                await self.client.disconnect()
                continue
            except BleakError as e:
                print(f"BLEAK ERROR:\t{e}")
                if self.client.is_connected:
                    await self.client.disconnect()
                continue
            except Exception as e:
                print(f"SEND ERROR with" + str(self.id) + e)  
            await asyncio.sleep(0.05)
        if self.finished:
            await self.disconnect()

    async def setup(self):
        if not self.client.is_connected:
            print("doing ble setup")
            while not self.client.is_connected:
                try:
                    print(f"Trying to connect to {self.address}")
                    await self.client.connect()
                    print("Connected to device")
                except KeyboardInterrupt as e:
                    sys.exit(0)
                except BleakError as e:
                    print(f"not found:\t{e}")
                    await self.setup()
        else:
            return     
        return