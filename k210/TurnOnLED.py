# 引入硬件库
import utime
from Maix import GPIO
from fpioa_manager import fm

io_led_g = 12
io_led_r = 13
io_led_b = 14
# 注册外设资源到对应的引脚
fm.register(io_led_g,fm.fpioa.GPIO0)
fm.register(io_led_r,fm.fpioa.GPIO1)
fm.register(io_led_b,fm.fpioa.GPIO2)


# 定义GPIO并设置工作模式
led_r=GPIO(GPIO.GPIO0,GPIO.OUT)
led_g=GPIO(GPIO.GPIO1,GPIO.OUT)
led_b=GPIO(GPIO.GPIO2,GPIO.OUT)

while True:
    led_r.value(0) 
    led_g.value(1) 
    led_b.value(1) 
    utime.sleep_ms(500)
    led_r.value(1)
    led_g.value(0) 
    led_b.value(1) 
    utime.sleep_ms(500)
    led_r.value(1) 
    led_g.value(1) 
    led_b.value(0) 
    utime.sleep_ms(500)

