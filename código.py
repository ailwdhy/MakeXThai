# agregar stop entre tiempos
# agregar indicardor de haber agarrado amarillo

import time
from novapi import * 
from mbuild.ranging_sensor import ranging_sensor_class
from mbuild.dual_rgb_sensor import dual_rgb_sensor_class
from mbuild.encoder_motor import encoder_motor_class
from mbuild import power_expand_board
from mbuild.smartservo import smartservo_class
from mbuild import gamepad


# Ks
kP = 35  # bajo 0.15 le cuesta mucho avanzar, claro que se debe al nano power
kD = 0.5
kI = 0

MIN_ERROR = 0.5  # cm

# Ks Servo Garra
kPS = 1
MIN_ERROR_S = 4  # grados

# Ks Garra
kPG = 18
kDG = 3
kIG = 1
MIN_ERROR_G = 1
expelled_error = 0

# WHEEL
# DIAMETER = 6.56  # cm
DIAMETER = 7  # cm

#   MEASURES
ROBOT_HEIGHT = 41  # intake to back
ROBOT_WIDTH = 41.1  # garra to back
ROBOT_DIAMETER = (ROBOT_WIDTH * 2 + ROBOT_HEIGHT * 2) ** 0.5  # CM
ROBOT = 38.4
# ESPA = ROBOT / 2 + (ROBOT * 2 + ROBOT_HEIGHT * 2) ** 0.5 / 2 + 2.075
ESPA = 11
# TO_LEFT = 12.25 + 12 + 20 + 12 - ROBOT/2
TO_LEFT = 12.25 + 12 + 20 + 12 - ROBOT/2 + 23 + 12 + 14  # faltan 12 + 10 + 4
# TO_FRONT = -(15 + 50 + 20/2) - ROBOT - ESPA + 209.25
TO_FRONT = 95.85 + 50 + ESPA * 0.4 - 1
LITTLE_DIS = 20 + 12
BIG_DIS = 23 + 12
#   POWERS
NANO_POWER = 10  # poder minimo para vencer la reduccion
MIN_POWER = 53
MAX_POWER = 53
TERA_POWER = 100

# GRADES
RPM_S = 200
BOOT_G = 128  # angulo en que inicia la garra, es una posicion
CLOSE_G = -43  # 27 ## debe ser una diferencia de posicion
OPEN_G = 43  # -27 ;   solo para diferencia cuando abrimos y cuando cerramos mejor

# TURN TIMES
TIME_90_DEG_I = 0.67  # 460
# TIME_90_DEG_D = 0.395
TIME_90_DEG_D = 0.67  # 460
TIME_90_DEG_D_START = 0.500
TIME_180_DEG = 1.28

#   MOTORES ENCODER
fl = encoder_motor_class("M1", "INDEX1")
fr = encoder_motor_class("M2", "INDEX1")
bl = encoder_motor_class("M3", "INDEX1")
br = encoder_motor_class("M4", "INDEX1")

#   DC MOTOR DEFINITION
TELES1 = "DC3"  # PUERTO DEL MOTOR QUE SUBE LA GARRA
TELES2 = "DC5"  # PUERTO DEL MOTOR 2 QUE SUBE LA GARRA, debe subir al mismo tiempo que 3, izq con garra frente
TELES3 = "DC6"  # PUERTO DEL MOTOR 3 QUE SUBE LA GARRA, debe subir al mismo tiempo que 2, der con garra frente
ELEVADOR_GARRA = "DC1"
M_SUBIR_MANOS = "DC1"
M_BANDA_2 = "DC2"
M_BANDA_1 = "DC3"
BANDERA = "DC4"

garra_s = smartservo_class("M6", "INDEX1")
Lanzador_s = smartservo_class("M5", "INDEX1")  # rotar
kpot = 0.8
#   SENSORES
rgb_sensor = dual_rgb_sensor_class("PORT4", "INDEX1")
distance_sensor = ranging_sensor_class("PORT1", "INDEX1")
dis_der_sen = ranging_sensor_class("PORT5", "INDEX1")
dis_izq_sen = ranging_sensor_class("PORT5", "INDEX2")
# bluetooth upload conectado en 5
# bluetooth gamepad conectado en 4

#   STOP_TIMES
# 1 or 0 para desactivar o activar los brushless
girar_brush = 0
STOP_TIME = 0.5
#   TEST ROUTINES


def holly_jolly_christmas(times=5):
    for i in range(times):
        rgb_sensor.set_led_color("green")
        time.sleep(0.1)
        rgb_sensor.set_led_color("red")
        time.sleep(0.1)
        rgb_sensor.set_led_color("blue")
        time.sleep(0.1)

#   MAIN CONTROL ROUTINES


def powerAll(flP, frP, blP, brP):
    fl.set_power(flP)
    fr.set_power(-frP)
    bl.set_power(blP)
    br.set_power(-brP)


def avanzar(power=MIN_POWER):
    powerAll(power, power, power, power)


def retroceder(power=MIN_POWER):
    powerAll(-power, -power, -power, -power)


def izquierda(power=MIN_POWER):
    powerAll(-power, power, power, -power * 1.2)


def derecha(power=MIN_POWER):
    powerAll(power, -power, -power, power * 0.8)


def girarIzquierda(power=60):
    powerAll(power, -power, power, -power)


def girarDerecha(power=60):
    powerAll(-power, power, -power, power)


def stop(sl=0):
    powerAll(0, 0, 0, 0)
    time.sleep(sl)

#   PID ROUTINES


def max_autonomo(potencia):
    if potencia > 70:
        potencia = 70
    elif potencia < -70:
        potencia = -70
    return potencia


# poner en paralelo la pared frente al robot
def paralelar():
    dd = dis_izq_sen.get_distance()
    di = dis_der_sen.get_distance()
    delta = di - dd
    delta = 0.55
    l_delta = 0

    end = False

    while abs(delta) >= 0.5:
        if timer() > 29:
            end = True
            break

        if dis_der_sen == 0.0 or dis_der_sen == 200 or dis_izq_sen == 0 or dis_izq_sen == 200:
            break
        # el los sensores de dis los inverti para corregir al sentido correcto
        dd = dis_izq_sen.get_distance()
        di = dis_der_sen.get_distance()
        delta = di - dd

        f_power = 4 * delta + (delta - l_delta) * 0.5

        f_power += (delta + l_delta) * 0

        if abs(delta) <= 0.5:
            f_power = 0
            stop()
            time.sleep(0.1)
            dd = dis_izq_sen.get_distance()
            di = dis_der_sen.get_distance()
            delta = di - dd

        if f_power < -50:
            f_power = -50
        if f_power > 50:
            f_power = 50

        if f_power < 30 and f_power > 0:
            f_power = 30
        elif f_power > -30 and f_power < 0:
            f_power = -30

        l_delta = delta

        delta = di - dd

        fl.set_power(-f_power)
        bl.set_power(-f_power)
        fr.set_power(-f_power)
        br.set_power(-f_power)

    stop()

    if end:
        Manual()


Kx = 8


def alinear():
    terminado = False
    global Kx
    fact_c1 = 1
    fact_c2 = 0
    K = 12
    s1 = dis_der_sen .get_distance() + fact_c1
    s2 = dis_izq_sen .get_distance() + fact_c2
    while (terminado == False):
        s1 = dis_der_sen .get_distance() + fact_c1
        s2 = dis_izq_sen .get_distance() + fact_c2
        dif = s1 - s2
        power_ang = dif * K
        powerM1 = -power_ang
        powerM2 = +power_ang
        powerM3 = -power_ang
        powerM4 = +power_ang
        fl.set_speed(max_autonomo(powerM1))
        fr.set_speed(-max_autonomo(powerM2))
        bl.set_speed(max_autonomo(powerM3))
        br.set_speed(-max_autonomo(powerM4))
        if dif > -1.5 and dif < 1.5:
            terminado = True

    fl.set_speed(0)
    fr.set_speed(0)
    bl.set_speed(0)
    br.set_speed(0)


def go_left(distance):
    # initial encoder es el angulo desde que se mide el error
    initial_encoder = fl.get_value("angle")

    error = 0.001  # valor solo para que corra el bucle almenos una vez
    last_error = 0
    distance = -distance
    while abs(error) > 0:

        error = get_error(distance, initial_encoder)

        f_power = - get_P(error) + get_D(error, last_error)

        if error == 0:
            stop()
            time.sleep(0.2)
            error = get_error(distance, initial_encoder)

        powerAll(-f_power, f_power * 1.6, f_power * 1.4, -f_power)


def run_to(distance):  # run using PID; distance [cm]
    # initial encoder es el angulo desde que se mide el error
    distance = -distance
    initial_encoder = fl.get_value("angle")

    error = 0.001  # valor solo para que corra el bucle almenos una vez
    last_error = error

    distance = distance * 0.7  # 0.8825 = (Goal / Real)
    end = False

    while abs(error) > 0:
        if timer() >= 29:
            end = True
            break
        error = get_error(distance, initial_encoder)
        f_power = get_P(error) - get_D(error, last_error)

        if f_power < -40:
            f_power = -40
        if f_power > 40:
            f_power = 40

        # if error == 0:
        #    stop()
        #    time.sleep(0.2)
        #    error = get_error(distance, initial_encoder)

        fl.set_power(f_power)
        fr.set_power(-f_power)
        bl.set_power(f_power)
        br.set_power(-f_power)
        # powerAll(f_power, f_power, f_power, f_power)

        last_error = error
    if end:
        Manual()


def get_error(distance, initial_encoder=0):  # distance [cm]
    # No falta hacer la correccion de la reduccion

    angle_done = fl.get_value("angle") - initial_encoder
    distance_done = DIAMETER * 3.14159 * angle_done / 360
    error = distance - distance_done

    # Ignorar error minimo
    if abs(error) <= MIN_ERROR:
        return 0

    return error


def get_P(error):
    return error * kP


def get_D(error, l_error):
    return (error - l_error) * kD


def move_servo_to(angle):
    # initial encoder es el angulo desde que se mide el error
    initial_angle = garra_s.get_value("angle")
    error = 0.001  # valor solo para que corra el bucle almenos una vez
    while abs(error) > 0:
        error = get_error_servo(angle, initial_angle)
        f_power = error * kPS
        if error == 0:
            garra_s.set_power(0)
            time.sleep(0.2)
            error = get_error_servo(angle, initial_angle)

        garra_s.set_power(f_power)


def get_error_servo(angle, initial_encoder=0):  # distance [cm]
    angle_done = garra_s.get_value("angle") - initial_encoder
    error = angle - angle_done
    # Ignorar error minimo
    if abs(error) <= MIN_ERROR_S:
        return 0
    return error


def pos_garra_at(height):
    error = 0.001  # solo para que marque error
    l_error = 0

    height += expelled_error

    error = get_error_garra(height)
    end = False

    while abs(error) > 0:
        if timer() >= 29:
            end = True
            break
        error = get_error_garra(height)
        # nueva filosofia PID
        #
        f_power = error * kPG
        f_power += (error - l_error) * kDG
        f_power += (error + l_error) * kIG

        # if f_power < 0:
        #    time.sleep(0.1)

        if error == 0:
            power_expand_board.stop(ELEVADOR_GARRA)
            time.sleep(0.05)
            error = get_error_garra(height)
            f_power = 0

        power_expand_board.set_power(ELEVADOR_GARRA, f_power)
        l_error = error

    if end:
        Manual()


def get_error_garra(goal):
    global expelled_error
    actual_pos = distance_sensor.get_distance()
    error = goal - actual_pos
    if abs(error) < MIN_ERROR_G:
        expelled_error = error
        error = 0

    return error
#   AUTONOMOUS ROUTINES


def bajarTodos():
    # comenzamos con la garra hasta abajo con el robot justo delante del primer cubo

    holly_jolly_christmas(1)
    # distancias a a recorrrer para ir al siguiente cubo
    distances = [32 * 2, 35 * 2, 32 * 2]

    for distance in distances:
        bajarCubo(distance)


def bajarCubo(distance):
    TIME = 0.3

    # paralelar()
    check_time()

    pos_garra_at(25.4 + 2)
    check_time()
    pos_garra_at(25.4 + 2)
    check_time()
    time.sleep(TIME)
    check_time()

    run_to(ESPA * 1.5 + 2)
    check_time()
    time.sleep(TIME)
    check_time()

    # garra_s.move(CLOSE_G, 200)
    # move_servo_to(CLOSE_G)
    garra_s.set_power(100)
    check_time()
    time.sleep(0.5)
    check_time()
    garra_s.set_power(5)
    check_time()
    time.sleep(TIME)
    check_time()

    # pos_garra_at(32.4) # si es desde el suelo
    # pos_garra_at(8)
    power_expand_board.set_power(ELEVADOR_GARRA, 100)
    check_time()
    time.sleep(0.4)
    check_time()
    power_expand_board.stop(ELEVADOR_GARRA)
    check_time()
    time.sleep(TIME)
    check_time()

    run_to(-ESPA * 1.5)
    check_time()
    time.sleep(TIME)
    check_time()

    garra_s.set_power(-100)
    check_time()
    time.sleep(0.5)
    check_time()
    garra_s.set_power(0)
    check_time()
    time.sleep(TIME)
    check_time()

    izquierda(60)
    time.sleep(1.5)
    stop()
    Manual()

    girarDerecha(60)
    check_time()
    time.sleep(TIME_90_DEG_D)
    check_time()
    stop()
    check_time()
    time.sleep(TIME)
    check_time()
    # alinear()

    # garra_s.move(OPEN_G, 200)
    # move_servo_to(OPEN_G)
    garra_s.set_power(-100)
    check_time()
    time.sleep(0.5)
    check_time()
    garra_s.set_power(0)
    check_time()
    time.sleep(TIME)
    check_time()

    girarDerecha(60)
    check_time()
    time.sleep(TIME_180_DEG)
    check_time()
    stop()
    check_time()
    time.sleep(TIME)
    check_time()
    # alinear()

    run_to(distance)
    check_time()
    time.sleep(TIME)
    check_time()

    girarDerecha(60)
    check_time()
    time.sleep(TIME_90_DEG_D)
    check_time()
    stop()
    check_time()
    time.sleep(TIME)
    check_time()

    # pos_garra_at(25.4 + 1)
    # time.sleep(1)


def check_time():
    if timer() >= 29:
        stop()
        Manual()
    ...


def cerrar_garra():
    MAX_CURRENT = 500
    MAX_VOLTAGE = 4.8
    while True:
        if garra_s.get_value("current") > MAX_CURRENT or garra_s.get_value("voltage") > MAX_VOLTAGE:
            break
        garra_s.set_power(80)

    garra_s.set_power(0)
    ...


def abrir_garra():
    # garra_s.set_power(-100) #MOVER SI EL SERVO VA ALREVEZ
    # time.sleep(0.5)
    # garra_s.set_power(0)
    #
    garra_s.move(-45)   #MOVER SI VA AL REVEZ
    garra_s.set_power(0)
    ...



def adminGarraPower(poweri):
    # control del servo para que no se queme

    MAX_CURRENT = 500
    MAX_VOLTAGE = 4.8
    MAX_TEMPERATURE = 38
    if (garra_s.get_value("current") > MAX_CURRENT and
        garra_s.get_value("voltage") > MAX_VOLTAGE and
            garra_s.get_value("temperature") >= MAX_TEMPERATURE):
        garra_s.set_power(poweri)
    else:
        garra_s.set_power(0)
    ...

# lava

#DISTANCIAS DE LA CANCHA
def Autonomous():
    reset_timer()
    reposo = 0.5
    lateral_a_primer_cubo = 33.45
    inicio_a_final_cancha = 184.25

    # inicia en la esquina derecha inferior, viendo a la izquierda (viendo al robot aliado)
    #
    rgb_sensor.set_led_color("red")
    # 3.5 como margen de error, porque choca con la dis pura
    run_to(TO_LEFT)
    time.sleep(reposo)

    girarDerecha(60)
    time.sleep(TIME_90_DEG_D)
    stop()
    time.sleep(reposo)

    # alinear()
    # time.sleep(reposo)

    run_to(TO_FRONT)
    time.sleep(reposo)

    derecha(100)
    time.sleep(0.6)
    stop()
    time.sleep(reposo)

    bajarTodos()

# asesamex01 - 2111090526
################################
#           MANUAL
###############################


def pararServo(smart_servo):
    current = 50
    if (smart_servo.get_value("current") > current):
        smart_servo.set_power(0)


def max(potencia):
    if potencia > 100:
        potencia = 100
    elif potencia < -100:
        potencia = -100
    return potencia


switch_garra_banda = 0
direction = 1

# manita

def todo():
    global kpot
    global direction
    global switch_garra_banda
    global auto

    # ----------------------Movimiento del chasis-----------------------
    lx = gamepad.get_joystick("Lx")
    ly = gamepad.get_joystick("Ly")
    rx = gamepad.get_joystick("Rx")
    pow_M1 = -ly + lx + rx
    pow_M2 = ly + lx + rx
    pow_M3 = -ly - lx + rx
    pow_M4 = ly - lx + rx
    time.sleep(0.001)
    fl.set_power(max(pow_M1 * kpot))
    fr.set_power(max(pow_M2 * kpot))
    bl.set_power(max(pow_M3 * kpot))
    br.set_power(max(pow_M4 * kpot))

    # Cambia el valor de la potencia para ir más rápido si se desea
    if gamepad.is_key_pressed("L_Thumb") or gamepad.is_key_pressed("R_Thumb"):
        kpot = 1
    else:
        kpot = 0.68

    # ----------------------Sistema de lanzadora-----------------------
    # Primer intake
    # Variables para controlar el estado del motor y su dirección
    motor_activado = False
    direccion = 1  # 1 para adelante, -1 para atrás

    # Bucle principal del programa
    while True:
        time.sleep(0.01)
        # Verificar si se ha presionado la tecla N2
        if gamepad.is_key_pressed("N2"):
            if not motor_activado:  # Si el motor no está activado, lo activamos
                power_expand_board.set_power(M_BANDA_1, 100)
                motor_activado = True
            else:  # Si el motor está activado, lo desactivamos
                power_expand_board.set_power(M_BANDA_1, 0)
                motor_activado = False

        # Verificar si se ha presionado la tecla N3
        if gamepad.is_key_pressed("N3"):
            if motor_activado:  # Si el motor está activado, cambiamos la dirección
                power_expand_board.set_power(M_BANDA_2, 100 * direccion)
                direccion *= -1

        # Banda de aeropuerto
        if gamepad.is_key_pressed("N1"):
            power_expand_board.set_power(M_BANDA_2, -100)
        elif gamepad.is_key_pressed("N4"):
            power_expand_board.set_power(M_BANDA_2, 100)
        else:
            power_expand_board.set_power(M_BANDA_2, 0)

        # ----------------------Sistema de manitas-----------------------
        if gamepad.is_key_pressed("Up"):
            power_expand_board.set_power(M_SUBIR_MANOS, -100)
        elif gamepad.is_key_pressed("Down"):
            power_expand_board.set_power(M_SUBIR_MANOS, 100)
        else:
            power_expand_board.set_power(M_SUBIR_MANOS, 0)

        # Girar el servo de ángulo de lanzadora
        if gamepad.is_key_pressed("R2"):
            Lanzador_s.set_power(10)  # original 15
            pararServo(Lanzador_s)
        elif gamepad.is_key_pressed("L2"):
            Lanzador_s.set_power(-10)
            pararServo(Lanzador_s)
        else:
            Lanzador_s.set_power(0)

        # Controla el gripper
        if gamepad.is_key_pressed("R1"):
            garra_s.set_power(-90)  # original 90
            pararServo(garra_s)
        elif gamepad.is_key_pressed("L1"):
            garra_s.set_power(90)
            pararServo(garra_s)
        else:
            garra_s.set_power(0)

        # ---------------- Controlar Bandera --------------------
        if gamepad.is_key_pressed("Right"):
            power_expand_board.set_power(BANDERA, 100)
        elif gamepad.is_key_pressed("Left"):
            power_expand_board.set_power(BANDERA, -100)
        else:
            power_expand_board.stop(BANDERA)

        # ---------------- Activar Autonomo --------------------
        if gamepad.is_key_pressed("≡") and not auto:
            auto = True
            automate2()

        # --------------- Control Brushless ----------------------
        power_expand_board.set_power("BL1", girar_brush * 60)  # original 50
        power_expand_board.set_power("BL2", girar_brush * 60)


def boton(btn, func):
    if gamepad.is_key_pressed(btn):
        while gamepad.is_key_pressed(btn):
            time.sleep(.025)
            todo()
        func()
        time.sleep(.025)


def para_giro_brush():
    global girar_brush
    if girar_brush == 0:
        girar_brush = 1
    elif girar_brush == 1:
        girar_brush = 0


auto = False


def Manual():
    while True:
        todo()
        boton("+", para_giro_brush)


# -----------------------  Tests ------------------------
def test_turn_times():
    girarIzquierda(60)
    time.sleep(TIME_90_DEG_I)
    stop()
    time.sleep(1)

    girarDerecha(60)
    time.sleep(TIME_90_DEG_D)
    stop()
    time.sleep(1)

    girarDerecha(60)
    time.sleep(TIME_180_DEG)
    stop()
    time.sleep(1)

    girarDerecha(60)
    time.sleep(TIME_180_DEG)
    stop()
    time.sleep(1)


def test_paralelar():
    paralelar()
    time.sleep(0.8)

    avanzar()
    time.sleep(0.3)
    retroceder()
    time.sleep(0.3)
    stop()
    time.sleep(0.8)


########################################################
def automate2():
    reset_timer()
    reposo = 0.25

    # Inicia en la esquina derecha inferior, viendo a la izquierda (viendo al robot aliado)
    rgb_sensor.set_led_color("red")
    # 3.5 como margen de error, porque choca con la dis pura
    run_to(TO_LEFT * 0.4 + 4)
    time.sleep(reposo)

    girarDerecha(60)
    time.sleep(TIME_90_DEG_D)
    stop()
    time.sleep(reposo)

    run_to(TO_FRONT * 0.28)
    time.sleep(reposo)

    girarIzquierda(60)
    time.sleep(TIME_90_DEG_I)
    stop()
    time.sleep(reposo)

    run_to(TO_LEFT * 0.6 + 15 + 1.5 - 29 + 0.3)
    time.sleep(reposo)

    girarDerecha(60)
    time.sleep(TIME_90_DEG_D)
    stop()
    time.sleep(reposo)

    run_to(TO_FRONT * 0.72 - 9.5 - 2)
    time.sleep(reposo)

    stop()
    time.sleep(reposo)

    bajarTodos()


while True:
    Manual()