import board
import busio
import adafruit_bno080
from adafruit_debug_i2c import DebugI2C

i2c = board.I2C()
i2c = DebugI2C(i2c)
bno = adafruit_bno080.BNO080(i2c)
print("**********88 OUT OF RESET *********")
