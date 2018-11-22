# *******************************************************************************************************
# Селетков И.П. 2018 0415.                                                                              *
# Считывание температуры с датчика ds1620.                                                              *
# Пример взят из github                                                                                 *
# https://github.com/rsolomon/py-DS1620                                                                 *
# *******************************************************************************************************
import traceback
import sys
# Импортируем библиотеку по работе с GPIO
import RPi.GPIO as GPIO
# Импортируем класс для работы со временем
import time
import asyncio
# Импортируем класс для работы с датчиком ds1620
from drivers.ds1620driver import DS1620
from fuzzy.controller import CFuzzyController
from nats.aio.client import Client as NATS
from nats.aio.errors import ErrConnectionClosed, ErrTimeout, ErrNoServers

def main():
    print("Начало работы программы.")
    # === Инициализация пинов для светодиода ===
    GPIO.setmode(GPIO.BCM)
    pin_led = 5
    pin_switch = 4
    GPIO.setup(pin_led, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(pin_switch, GPIO.OUT, initial=GPIO.HIGH)

    # Инициализация пинов для датчика температуры.
    # rst, dq, clk
    t_sensor = DS1620(17, 18, 27)

    controller = CFuzzyController(50)

    # Период переключения нагревателя, сек.
    period = 10
    time1 = 0
    time2 = 0
    time3 = -10

    # Состояние системы. Активирован нагреватель (0) или нет (1).
    state = 1
    # Бесконечный цикл для измерения температуры.
    while 1:
        if state==0:
            if time2==0:
                state = 1
                continue

            # Изменяем состояние светодиода
            GPIO.output(pin_led,  GPIO.LOW)
            GPIO.output(pin_switch,  GPIO.HIGH)
            state = 1
            time.sleep(time2)

            continue
        time3 += period

        # Считываем текущую температуру.
        temperature = t_sensor.get_temperature()
        # Выводим температуру на экран.
        print("| "+str(time3)+" | Температура: " + str(temperature)+" | ", end='')

        # Подключаемся к серверу NATS для передачи данных
        if time3 == 10:
            async def run(loop):
                nc = NATS()
                await nc.connect("nats://192.168.1.104:4222", loop=loop)
                print(nc.is_connected)


        # Рассчитываем значения мощностей нагревателя и охладителя для данной температуры.
        heater, cooler = controller.get(temperature)
        # Выводим мощности на экран
        print("Нагреватель: " + str(heater)+" | ", end='')
        print("Охладитель: " + str(cooler) + " |")

        GPIO.output(pin_switch, GPIO.LOW)
        GPIO.output(pin_led, GPIO.HIGH)
        state = 0
        # Ожидаем одну секунди и переходим к повтору операции.
        time1 = period * heater * 0.01
        time2 = period - time1
        time.sleep(time1)
try:
    main()

except KeyboardInterrupt:
    # Выход из программы по нажатию Ctrl+C
    print("Завершение работы Ctrl+C.")
except Exception as e:
    # Прочие исключения
    print("Ошибка в приложении.")
    # Подробности исключения через traceback
    traceback.print_exc(limit=2, file=sys.stdout)
finally:
    print("Сброс состояния порта в исходное.")
    # Возвращаем пины в исходное состояние
    GPIO.cleanup()
    # Информируем о завершении работы программы
    print("Программа завершена.")
