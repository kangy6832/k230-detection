import utime
from Maix import GPIO
from fpioa_manager import fm

io_led_green = 12
io_led_red = 13
io_led_blue = 14

io_boot_key = 16

fm.register(io_led_green, fm.fpioa.GPIO0)
fm.register(io_led_red, fm.fpioa.GPIO1)
fm.register(io_led_blue, fm.fpioa.GPIO2)

fm.register(io_boot_key, fm.fpioa.GPIOHS0)

# fm.register(io_boot_key, fm.fpioa.GPIO0)

led_red = GPIO(GPIO.GPIO1, GPIO.OUT)
led_green = GPIO(GPIO.GPIO0, GPIO.OUT)
led_blue = GPIO(GPIO.GPIO2, GPIO.OUT)

boot_key = GPIO(GPIO.GPIOHS0, GPIO.IN, GPIO.PULL_UP)

# boot_key = GPIO(GPIO.GPIO0, GPIO.IN)

led_blue.value(1)
led_green.value(1)
led_red.value(1)

led_state = True

def test_irq(key):
    global led_state
    utime.sleep_ms(20) # 消抖
    if key.value() == 0:
        led_state = not led_state # not 取反
        led_green.value(led_state)

boot_key.irq(test_irq, GPIO.IRQ_FALLING) # 中断触发方式（只用注册一次）

# while True:
    # if boot_key.value() == 0:
    #     utime.sleep_ms(20) # 消抖

    #     if boot_key.value() == 0:
    #         led_state = not led_state # not 取反
    #         led_green.value(led_state)

    #     while(boot_key.value() == 0):
    #         utime.sleep_ms(20)
