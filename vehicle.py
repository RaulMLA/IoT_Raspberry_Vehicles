import json
import RPi.GPIO as GPIO
import signal
import sys
import threading
from threading import Semaphore
import time


def setup() -> None:
    global threads

    global executed_command
    global should_run

    global sem

    global dc_motor_object
    global dc_speed
    global dc_time
    global interval
    global deceleration

    global LDR_PIN
    global light_count

    global TRIGGER_PIN
    global ECHO_PIN
    global obstacle_detected

    global BUTTON_PIN

    global SERVO_PIN
    global servo_angle

    global vehicle_control_commands    

    global pending_execution

    global servo_object

    global pwm_rear_right
    global pwm_rear_left

     # Comando actual pendiente de ejecución.
    pending_execution = False

    # Detección de obstáculos.
    obstacle_detected = False

    # Pool de hilos.
    threads = []

    # Control de ejecución.
    executed_command = True
    should_run = False

    # Semáforo.
    sem = Semaphore(1)

    # Modo de numeración para los pines.
    GPIO.setmode(GPIO.BCM)

    # Desactivar advertencias.
    GPIO.setwarnings(False)

    # Motor CC.
    MOTOR_A_PIN = 5
    MOTOR_B_PIN = 6
    MOTOR_E_PIN = 13

    dc_speed = 0
    dc_time = 0
    interval = 0.3
    deceleration = False

    GPIO.setup(MOTOR_A_PIN, GPIO.OUT)
    GPIO.setup(MOTOR_B_PIN, GPIO.OUT)
    GPIO.setup(MOTOR_E_PIN, GPIO.OUT)

    GPIO.output(MOTOR_A_PIN, True)
    GPIO.output(MOTOR_B_PIN, False)
    GPIO.output(MOTOR_E_PIN, True)
    
    dc_motor_object = GPIO.PWM(MOTOR_E_PIN, 100)
    dc_motor_object.start(100)
    dc_motor_object.ChangeDutyCycle(0)

    # LDR.
    LDR_PIN = 4
    light_count = 1000

    # Sensor de ultrasonidos.
    TRIGGER_PIN = 25
    ECHO_PIN = 24
    
    GPIO.setup(TRIGGER_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)

    # Botón.
    BUTTON_PIN = 23
    
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=button_manager, bouncetime=100)

    # Servo.
    SERVO_PIN = 16
    servo_angle = 90.0

    GPIO.setup(SERVO_PIN, GPIO.OUT)
    servo_object = GPIO.PWM(SERVO_PIN, 50)
    servo_object.start(set_angle(servo_angle))
    
    # Leds.
    REAR_RIGHT_RED = 17
    REAR_RIGHT_GREEN = 27
    REAR_RIGHT_BLUE = 22

    REAR_LEFT_RED = 14
    REAR_LEFT_GREEN = 15
    REAR_LEFT_BLUE = 18

    GPIO.setup(REAR_RIGHT_RED, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(REAR_RIGHT_GREEN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(REAR_RIGHT_BLUE, GPIO.OUT, initial=GPIO.LOW)

    GPIO.setup(REAR_LEFT_RED, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(REAR_LEFT_GREEN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(REAR_LEFT_BLUE, GPIO.OUT, initial=GPIO.LOW)
    
    pwm_rear_right_red = GPIO.PWM(REAR_RIGHT_RED, 100)
    pwm_rear_right_green = GPIO.PWM(REAR_RIGHT_GREEN, 100)
    pwm_rear_right_blue = GPIO.PWM(REAR_RIGHT_BLUE, 100)

    pwm_rear_left_red = GPIO.PWM(REAR_LEFT_RED, 100)
    pwm_rear_left_green = GPIO.PWM(REAR_LEFT_GREEN, 100)
    pwm_rear_left_blue = GPIO.PWM(REAR_LEFT_BLUE, 100)

    pwm_rear_right_red.start(0)
    pwm_rear_right_green.start(0)
    pwm_rear_right_blue.start(0)

    pwm_rear_left_red.start(0)
    pwm_rear_left_green.start(0)
    pwm_rear_left_blue.start(0)

    pwm_rear_right = [pwm_rear_right_red, pwm_rear_right_green, pwm_rear_right_blue]
    pwm_rear_left = [pwm_rear_left_red, pwm_rear_left_green, pwm_rear_left_blue]

    # Carga de comandos.
    load_commands()

    print('La configuración de los dispositivos ha finalizado.')


def stop_pwm_objects() -> None:
    dc_motor_object.stop()
    servo_object.stop()

    for i in range(3):
        pwm_rear_right[i].stop()
        pwm_rear_left[i].stop()


def pin_on(pin, intensity: int) -> None:
    pin.ChangeDutyCycle(intensity)


def pin_off(pin) -> None:
    pin.ChangeDutyCycle(0)


def red(red_pin, green_pin, blue_pin, intensity:int = 100) -> None:
    pin_on(red_pin, intensity)
    pin_off(green_pin)
    pin_off(blue_pin)
    

def yellow(red_pin, green_pin, blue_pin, intensity:int = 100) -> None:
    pin_on(red_pin, intensity)
    pin_on(green_pin, intensity)
    pin_off(blue_pin)


def turn_off(red_pin, green_pin, blue_pin) -> None:
    pin_off(red_pin)
    pin_off(green_pin)
    pin_off(blue_pin)


def lights() -> None:
    print('** Iniciando hilo de luces.')

    while should_run:
        # Giro a izquierda.
        if servo_angle > 100 and not obstacle_detected:
            turn_off(*pwm_rear_right)
            for i in range(3):
                yellow(*pwm_rear_left)
                time.sleep(0.15)
                turn_off(*pwm_rear_left)
                time.sleep(0.15)
        # Giro a derecha.
        elif servo_angle < 80 and not obstacle_detected:
            turn_off(*pwm_rear_left)
            for i in range(3):
                yellow(*pwm_rear_right)
                time.sleep(0.15)
                turn_off(*pwm_rear_right)
                time.sleep(0.15)
        else:
            if light_count > 2000:
                # Frenado con luz baja.
                if deceleration and dc_time > 0:
                    red(*pwm_rear_right)
                    red(*pwm_rear_left)
                # Luz baja.
                else:
                    red(*pwm_rear_right, 50)
                    red(*pwm_rear_left, 50)
            else:
                # Frenado con luz alta.
                if deceleration and dc_time > 0:
                    red(*pwm_rear_right, 50)
                    red(*pwm_rear_left, 50)
                # Luz alta.
                else:
                    turn_off(*pwm_rear_right)
                    turn_off(*pwm_rear_left)
        
    turn_off(*pwm_rear_right)
    turn_off(*pwm_rear_left)


def set_angle(angle: float) -> float:
    global servo_object

    angle = max(0, min(180, angle))
    start = 4
    end = 12.5
    ratio = (end - start) / 180
    angle_as_percent = start + angle * ratio

    return angle_as_percent


def move_servo(angle: float) -> None:
    servo_object.start(set_angle(angle))


def load_commands() -> None:
    global vehicle_control_commands
    with open('./commands.json') as json_file:
        vehicle_control_commands = json.load(json_file)
    vehicle_control_commands.append( {'SteeringAngle': 90.0, 'Speed': 0.0, 'Time': 3.0})


def execute_command(command: dict) -> None:
    global pending_execution
    global dc_speed
    global dc_time
    global servo_angle
    global deceleration

    print(f'Ejecutando comando: {command}')

    deceleration = True if command['Speed'] < dc_speed else False

    servo_angle = command['SteeringAngle']
    dc_speed = command['Speed']
    dc_time = command['Time']

    pending_execution = True

    move_servo(servo_angle)


def control_vehicle() -> None:
    print('** Iniciando hilo de control.')

    global vehicle_control_commands

    while len(vehicle_control_commands) > 0 and should_run:
        if not pending_execution:
            command = vehicle_control_commands[0]
            vehicle_control_commands.pop(0)
            execute_command(command)

    
def move_dc_motor() -> None:
    print('** Iniciando hilo de motor.')

    global dc_speed
    global dc_time
    global pending_execution
    global obstacle_detected
    global deceleration

    while should_run:
        while obstacle_detected and should_run:
            deceleration = True if dc_speed > 0 else False
            dc_motor_object.ChangeDutyCycle(0)
        if obstacle_detected and not should_run:
            pending_execution = False
            break
        if pending_execution and dc_time >= 0:
            start_time = time.time()
            dc_motor_object.ChangeDutyCycle(dc_speed)
            time.sleep(interval)
            elapsed_time = time.time() - start_time
            dc_time -= elapsed_time
            if dc_time <= 0:
                dc_time = 0
                pending_execution = False

    if dc_time > 0:
        vehicle_control_commands.insert(0, {'SteeringAngle': servo_angle, 'Speed': dc_speed, 'Time': dc_time})

    dc_motor_object.ChangeDutyCycle(0)


def measure_light() -> None:
    print('** Iniciando hilo de luz.')

    global light_count

    while should_run:
        
        counter = 0

        GPIO.setup(LDR_PIN, GPIO.OUT)
        GPIO.output(LDR_PIN, GPIO.LOW)
        time.sleep(0.1)
        GPIO.setup(LDR_PIN, GPIO.IN)

        while (GPIO.input(LDR_PIN) == GPIO.LOW):
            counter += 1

        light_count = counter

        print('Luz:', light_count) if should_run else None


def measure_distance() -> None:
    print('** Iniciando hilo de distancia.')

    global obstacle_detected

    while should_run:
        GPIO.output(TRIGGER_PIN, True)
        time.sleep(0.00001)
        GPIO.output(TRIGGER_PIN, False)

        pulse_start_time = time.time()
        pulse_end_time = time.time()

        while GPIO.input(ECHO_PIN) == 0 and (time.time() - pulse_start_time) < 0.1:
            pulse_start_time = time.time()

        while GPIO.input(ECHO_PIN) == 1 and (time.time() - pulse_end_time) < 0.1:
            pulse_end_time = time.time()

        pulse_duration = pulse_end_time - pulse_start_time
        distance = (pulse_duration * 34300) / 2

        obstacle_detected = True if distance < 20 and distance >= 0 else False

        if should_run and distance >= 0:
            print('Distancia:', round(distance, 2), 'cm')
        
        time.sleep(1)


def button_manager(channel) -> None:
    global should_run
    global executed_command

    if not GPIO.input(BUTTON_PIN):
        sem.acquire()
        should_run = not should_run
        print('------------------ ENCENDER ------------------') if should_run else print('------------------ APAGAR --------------------')
        executed_command = False
        sem.release()


def start_vehicle() -> None:
    print('............. Iniciando vehículo .............')

    threads.append(threading.Thread(target=lights, args=(), daemon=True))
    threads.append(threading.Thread(target=move_dc_motor, args=(), daemon=True))
    threads.append(threading.Thread(target=measure_light, args=(), daemon=True))
    threads.append(threading.Thread(target=measure_distance, args=(), daemon=True))
    threads.append(threading.Thread(target=control_vehicle, args=(), daemon=True))

    for i in range(len(threads)):
        threads[i].start()


def shutdown_vehicle() -> None:
    print('............ Deteniendo vehículo ............')

    for i in range(len(threads)):
        threads[i].join()

        if i == 0:
            print('** Hilo de luces terminado.')
        elif i == 1:
            print('** Hilo de motor terminado.')
        elif i == 2:
            print('** Hilo de luz terminado.')
        elif i == 3:
            print('** Hilo de distancia terminado.')
        elif i == 4:
            print('** Hilo de control terminado.')

    threads.clear()

    print('Tiempo de comando pendiente:', dc_time)

    print('............. Vehículo detenido .............')



if __name__ == '__main__':

    global executed_command

    try:
        setup()

        while True:
            sem.acquire()
            if should_run and not executed_command:
                start_vehicle()
                executed_command = True
            elif not should_run and not executed_command:
                shutdown_vehicle()
                executed_command = True
            sem.release()

    except KeyboardInterrupt:
        print('Programa detenido.')

    finally:
        stop_pwm_objects()
        GPIO.cleanup()
        sys.exit(0)
