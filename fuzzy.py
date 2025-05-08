import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import lgpio
import time

# === Define Inputs ===
flame_center = ctrl.Antecedent(np.arange(0, 2, 1), 'Flame Center')  # 0 or 1

error = ctrl.Antecedent(np.arange(-1, 1.1, 0.1), 'Error(Right-Left)')
temperature = ctrl.Antecedent(np.arange(0, 31, 0.5), 'Temperature')

# === Define Outputs ===
left_motor = ctrl.Consequent(np.arange(0, 101, 1), 'Left Motor')
right_motor = ctrl.Consequent(np.arange(0, 101, 1), 'Right Motor')

# === Membership functions for Flame Center ===
flame_center['Low'] = fuzz.trapmf(flame_center.universe, [-1, -1, 0, 1])
flame_center['High'] = fuzz.trapmf(flame_center.universe, [0, 1, 2, 2])

# === Membership functions for Error ===
error['Left'] = fuzz.trapmf(error.universe, [-1, -1, -0.5, 0])
error['Center'] = fuzz.trimf(error.universe, [-0.5, 0, 0.5])
error['Right'] = fuzz.trapmf(error.universe, [0, 0.5, 1, 1])

# === Membership functions for Temperature ===
temperature['Far'] = fuzz.trapmf(temperature.universe, [20, 25, 30, 30])
temperature['Mid'] = fuzz.trimf(temperature.universe, [15, 20, 25])
temperature['Near'] = fuzz.trapmf(temperature.universe, [0, 0, 15, 20])

# === Membership functions for Motors ===
left_motor['Stop'] = fuzz.trapmf(left_motor.universe, [0, 0, 0, 0])
left_motor['Slow'] = fuzz.trimf(left_motor.universe, [45, 55, 65])
left_motor['Medium'] = fuzz.trimf(left_motor.universe, [55, 65, 75])
left_motor['Fast'] = fuzz.trapmf(left_motor.universe, [65, 75, 100, 100])

right_motor['Stop'] = fuzz.trapmf(right_motor.universe, [0, 0, 0, 0])
right_motor['Slow'] = fuzz.trimf(right_motor.universe, [30, 45, 60])
right_motor['Med'] = fuzz.trimf(right_motor.universe, [45, 60, 75])
right_motor['Fast'] = fuzz.trapmf(right_motor.universe, [60, 75, 100, 100])

# === Rules ===
rule1 = ctrl.Rule(flame_center['Low'] & error['Left'] & temperature['Far'], (left_motor['Medium'], right_motor['Fast']))
rule2 = ctrl.Rule(flame_center['Low'] & error['Left'] & temperature['Mid'], (left_motor['Slow'], right_motor['Med']))
rule3 = ctrl.Rule(flame_center['Low'] & error['Left'] & temperature['Near'], (left_motor['Stop'], right_motor['Slow']))
rule4 = ctrl.Rule(flame_center['High'] & error['Left'] & temperature['Far'], (left_motor['Medium'], right_motor['Fast']))
rule5 = ctrl.Rule(flame_center['High'] & error['Left'] & temperature['Mid'], (left_motor['Slow'], right_motor['Med']))
rule6 = ctrl.Rule(flame_center['High'] & error['Left'] & temperature['Near'], (left_motor['Stop'], right_motor['Slow']))
rule7 = ctrl.Rule(flame_center['Low'] & error['Center'] & temperature['Far'], (left_motor['Fast'], right_motor['Fast']))
rule8 = ctrl.Rule(flame_center['Low'] & error['Center'] & temperature['Mid'], (left_motor['Medium'], right_motor['Med']))
rule9 = ctrl.Rule(flame_center['Low'] & error['Center'] & temperature['Near'], (left_motor['Medium'], right_motor['Med']))
rule10 = ctrl.Rule(flame_center['High'] & error['Center'] & temperature['Far'], (left_motor['Medium'], right_motor['Med']))
rule11 = ctrl.Rule(flame_center['High'] & error['Center'] & temperature['Mid'], (left_motor['Slow'], right_motor['Slow']))
rule12 = ctrl.Rule(flame_center['High'] & error['Center'] & temperature['Near'], (left_motor['Stop'], right_motor['Stop']))
rule13 = ctrl.Rule(flame_center['Low'] & error['Right'] & temperature['Far'], (left_motor['Fast'], right_motor['Med']))
rule14 = ctrl.Rule(flame_center['Low'] & error['Right'] & temperature['Mid'], (left_motor['Medium'], right_motor['Slow']))
rule15 = ctrl.Rule(flame_center['Low'] & error['Right'] & temperature['Near'], (left_motor['Slow'], right_motor['Stop']))
rule16 = ctrl.Rule(flame_center['High'] & error['Right'] & temperature['Far'], (left_motor['Fast'], right_motor['Med']))
rule17 = ctrl.Rule(flame_center['High'] & error['Right'] & temperature['Mid'], (left_motor['Medium'], right_motor['Slow']))
rule18 = ctrl.Rule(flame_center['High'] & error['Right'] & temperature['Near'], (left_motor['Slow'], right_motor['Stop']))

motor_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6,
                                 rule7, rule8, rule9, rule10, rule11, rule12,
                                 rule13, rule14, rule15, rule16, rule17, rule18])
motor_sim = ctrl.ControlSystemSimulation(motor_ctrl)

# === GPIO Setup ===
CENTER_SENSOR_PIN = 10
LEFT_SENSOR_PIN = 9
RIGHT_SENSOR_PIN = 20
TRIG = 24
ECHO = 25
RIGHT_B1 = 23
RIGHT_A1 = 22
RIGHT_SPEED = 12
LEFT_A1 = 27
LEFT_B1 = 17
LEFT_SPEED = 13
SERVO_PIN = 18
WATER_SENSOR_PIN = 16
# Water pump pins
IN1 = 5   # GPIO 5 for forward direction
IN2 = 6   # GPIO 6 for backward direction
ENA = 19  # GPIO 19 for PWM (speed control)

h = lgpio.gpiochip_open(0)

# Claim all GPIO pins
lgpio.gpio_claim_input(h, CENTER_SENSOR_PIN)
lgpio.gpio_claim_input(h, LEFT_SENSOR_PIN)
lgpio.gpio_claim_input(h, RIGHT_SENSOR_PIN)
lgpio.gpio_claim_output(h, TRIG)
lgpio.gpio_claim_input(h, ECHO)
lgpio.gpio_claim_input(h, WATER_SENSOR_PIN)
lgpio.gpio_claim_output(h, RIGHT_A1)
lgpio.gpio_claim_output(h, RIGHT_B1)
lgpio.gpio_claim_output(h, RIGHT_SPEED)
lgpio.gpio_claim_output(h, LEFT_A1)
lgpio.gpio_claim_output(h, LEFT_B1)
lgpio.gpio_claim_output(h, LEFT_SPEED)
lgpio.gpio_claim_output(h, SERVO_PIN)
# Water pump pins
lgpio.gpio_claim_output(h, IN1)
lgpio.gpio_claim_output(h, IN2)
lgpio.gpio_claim_output(h, ENA)

# Initialize PWM
lgpio.tx_pwm(h, RIGHT_SPEED, 1000, 0)
lgpio.tx_pwm(h, LEFT_SPEED, 1000, 0)
lgpio.tx_pwm(h, SERVO_PIN, 50, 0)
lgpio.tx_pwm(h, ENA, 1000, 0)  # Initialize water pump PWM

def set_angle(angle):
    duty = 2.5 + (angle / 180) * 10
    lgpio.tx_pwm(h, SERVO_PIN, 50, duty)
    time.sleep(0.5)
    lgpio.tx_pwm(h, SERVO_PIN, 50, 0)

def get_distance():
    lgpio.gpio_write(h, TRIG, 0)
    time.sleep(0.0001)
    lgpio.gpio_write(h, TRIG, 1)
    time.sleep(0.00001)
    lgpio.gpio_write(h, TRIG, 0)

    timeout = time.time() + 0.02
    while lgpio.gpio_read(h, ECHO) == 0:
        pulse_start = time.time()
        if time.time() > timeout:
            return 20

    timeout = time.time() + 0.02
    while lgpio.gpio_read(h, ECHO) == 1:
        pulse_end = time.time()
        if time.time() > timeout:
            return 20

    duration = pulse_end - pulse_start
    return round(duration * 17150, 2)

def start_water_pump(speed_percentage):
    """
    Start the water pump with specified speed (0 to 100% duty cycle).
    """
    print(f"Water pump started at {speed_percentage}% speed.")
    lgpio.gpio_write(h, IN1, 1)  # Forward direction
    lgpio.gpio_write(h, IN2, 0)
    lgpio.tx_pwm(h, ENA, 1000, speed_percentage)  # Update PWM duty cycle

def stop_water_pump():
    """
    Stop the water pump.
    """
    print("Water pump stopped!")
    lgpio.gpio_write(h, IN1, 0)
    lgpio.gpio_write(h, IN2, 0)
    lgpio.tx_pwm(h, ENA, 1000, 0)  # Stop PWM

print("Starting system...")
time.sleep(2)

try:
    while True:
        center_val = lgpio.gpio_read(h, CENTER_SENSOR_PIN)
        left_val = lgpio.gpio_read(h, LEFT_SENSOR_PIN)
        right_val = lgpio.gpio_read(h, RIGHT_SENSOR_PIN)
        dist = get_distance()
        water_val = lgpio.gpio_read(h, WATER_SENSOR_PIN)

        if center_val == 0:
            print("🔥 Flame detected at CENTER!")
        if left_val == 0:
            print("🔥 Flame detected at LEFT!")
        if right_val == 0:
            print("🔥 Flame detected at RIGHT!")
        if center_val == 1 and left_val == 1 and right_val == 1:
            print("No flame detected.")

        if water_val == 1:
            print("✅ Water tank is FULL")
        else:
            print("⚠️ Water level LOW!")

        if dist != -1 and dist < 20:
            print(f"⛔ Obstacle at {dist} cm")
        else:
            print(f"✅ No obstacle. Distance = {dist} cm")

        flame_center_val = 1 if center_val == 0 else 0
        error_val = 0 if left_val == 0 and right_val == 0 else (-1 if left_val == 0 else (1 if right_val == 0 else 0))
        temp_val = dist if dist <= 30 else 30  

        print(f"Inputs: FlameCenter={flame_center_val}, Error={error_val}, Temp(Dist)={temp_val}")

        motor_sim.input['Flame Center'] = flame_center_val
        motor_sim.input['Error(Right-Left)'] = error_val
        motor_sim.input['Temperature'] = temp_val
        motor_sim.compute()

        left_output = motor_sim.output['Left Motor']
        right_output = motor_sim.output['Right Motor']

        print(f"Motor outputs: Left={left_output:.1f}%, Right={right_output:.1f}%")

        # Control motors
        lgpio.gpio_write(h, LEFT_A1, 1)
        lgpio.gpio_write(h, LEFT_B1, 0)
        lgpio.gpio_write(h, RIGHT_A1, 1)
        lgpio.gpio_write(h, RIGHT_B1, 0)
        lgpio.tx_pwm(h, LEFT_SPEED, 1000, left_output)
        lgpio.tx_pwm(h, RIGHT_SPEED, 1000, right_output)

        # Control water pump based on motor states
        if left_output <= 1 and right_output <= 1 and center_val ==0:
            start_water_pump(100)  # Turn on pump at full speed when both motors are stopped
        else:
            stop_water_pump()

        set_angle(0)
        time.sleep(0.5)
        set_angle(180)
        time.sleep(0.5)
        set_angle(90)

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    # Clean up all GPIO
    lgpio.tx_pwm(h, RIGHT_SPEED, 1000, 0)
    lgpio.tx_pwm(h, LEFT_SPEED, 1000, 0)
    lgpio.tx_pwm(h, ENA, 1000, 0)  # Stop water pump
    lgpio.gpiochip_close(h)
    print("Stopped & cleaned up.") 
