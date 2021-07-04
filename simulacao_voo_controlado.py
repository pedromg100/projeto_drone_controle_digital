################################################################################################################
#
#               DIGITAL CONTROL - EE/UFSCAR
#
#   Author: Pedro Moraes Gomes
#   Version: 1
#   Last-Update: 04.07.2021
#
#   Info: Digital control of quadricopter
#
#   Code developed by student taking the class to control
#   the proposed drone system as the final class project
#
################################################################################################################

import numpy as np
import matplotlib.pyplot as plt

"""Parâmetros"""
Ap = -20.97
Bp = 361.44
Cp = 2622.96
v = 11.1
l = 235 * 10 ** -6
g = 9.80665
Ixx = 2.85 * 10 ** -3
Iyy = 6.46 * 10 ** -3
Izz = 8.4 * 10 ** -1
m = 0.8

"""Dados usados no dimensionamento dos componentes"""
d_b_list = [(0.0000011, 0.0000542)]

""" Lista de dados para os 10 cenários de umidade, temperatura e pressão"""
# d_b_list = [
#     (0.00001360189968, 0.00008595234782),
#     (0.00001360189968, 0.00006971609194),
#     (0.00001360189968, 0.00005582481373),
#     (0.00001360189968, 0.00004197781059),
#     (0.00001360189968, 0.00004552975534),
#     (0.00001360052663, 0.00004802082839),
#     (0.00001360052663, 0.00005499900436),
#     (0.00001360052663, 0.00005389500631),
#     (0.00001360052663, 0.00004401497749),
#     (0.00001360052663, 0.00003473498418),
# ]

T = 0.1
total_simulation_time = 20

"""Constantes dos controladores"""
k_roll = 5
i_roll = 2
d_roll = 3
k_pitch = 5
i_pitch = 2
d_pitch = 3
k_height = 6
i_height = 3
d_height = 5
int_height_control_limit = 0.5

"""Listas de dados"""
height_list_of_vectors = []
time_list_of_vectors = []
roll_list_of_vectors = []
pitch_list_of_vectors = []
duty_cycle_1_list_of_vectors = []
duty_cycle_2_list_of_vectors = []
duty_cycle_3_list_of_vectors = []
duty_cycle_4_list_of_vectors = []

roll_ref = 0
pitch_ref = 0
height_ref = 1
flag_noise = False
"""Simulação para cada cenário"""
for d, b in d_b_list:
    """Condições iniciais"""
    omega_1 = 0
    omega_2 = 0
    omega_3 = 0
    omega_4 = 0
    omega_1_k_1 = 0
    omega_2_k_1 = 0
    omega_3_k_1 = 0
    omega_4_k_1 = 0
    roll = 0
    pitch = 0
    height = 0
    yaw = 0
    roll_k_1 = 0
    roll_k_2 = 0
    pitch_k_1 = 0
    pitch_k_2 = 0
    yaw_k_1 = 0
    yaw_k_2 = 0
    height_k_1 = 0
    height_k_2 = 0

    int_roll_control = 0
    int_pitch_control = 0
    int_height_control = 0
    control_roll_k_1 = 0
    control_pitch_k_1 = 0
    control_height_k_1 = 0
    control_omega_1_k_1 = 0
    control_omega_2_k_1 = 0
    control_omega_3_k_1 = 0
    control_omega_4_k_1 = 0

    height_vector = []
    time_vector = []
    roll_vector = []
    pitch_vector = []
    duty_cycle_1_vector = []
    duty_cycle_2_vector = []
    duty_cycle_3_vector = []
    duty_cycle_4_vector = []

    tempo = 0
    """Simulação no tempo"""
    while tempo < total_simulation_time:
        """Interface sistema_controlador
        Pegando dados atrasados, no instante (k-1)"""
        control_roll = roll_k_1
        roll_sensor_noise = np.random.normal(0, 0.005)
        if flag_noise:
            control_roll = control_roll * (1 + roll_sensor_noise)
        control_pitch = pitch_k_1
        pitch_sensor_noise = np.random.normal(0, 0.005)
        if flag_noise:
            control_pitch = control_pitch * (1 + pitch_sensor_noise)
        control_height = height_k_1
        height_sensor_noise = np.random.normal(0, 0.005)
        if flag_noise:
            control_height = control_height * (1 + height_sensor_noise)

        """Cálculo do controlador
        Utiliza como entrada os valores desejados e 
        os estados das variáveis controladas"""
        # Controladores PID
        roll_error = roll_ref - control_roll
        prop_roll_control = k_roll * roll_error
        int_roll_control += i_roll * roll_error * T
        der_roll_control = d_roll * (control_roll - control_roll_k_1) / T
        roll_control_signal = prop_roll_control + int_roll_control - der_roll_control
        control_U2 = roll_control_signal * Ixx

        pitch_error = pitch_ref - control_pitch
        prop_pitch_control = k_pitch * pitch_error
        int_pitch_control += i_pitch * pitch_error * T
        der_pitch_control = d_pitch * (control_pitch - control_pitch_k_1) / T
        pitch_control_signal = prop_pitch_control + int_pitch_control - der_pitch_control
        control_U3 = pitch_control_signal * Iyy

        height_error = height_ref - control_height
        prop_height_control = k_height * height_error
        int_height_control += i_height * height_error * T
        if int_height_control > int_height_control_limit:
            int_height_control = int_height_control_limit
        if int_height_control < -int_height_control_limit:
            int_height_control = -int_height_control_limit
        der_height_control = d_height * (control_height - control_height_k_1) / T
        height_control_signal = prop_height_control + int_height_control - der_height_control
        control_U1 = (height_control_signal + g) * m / (np.cos(pitch) * np.cos(roll))

        control_U4 = 0

        control_roll_k_1 = control_roll
        control_pitch_k_1 = control_pitch
        control_height_k_1 = control_height

        # Calculando velocidade hélices
        control_omega_1_square = control_U1 / (4 * b) - control_U3 / (2 * b * l) - control_U4 / (4 * d)
        control_omega_2_square = control_U1 / (4 * b) - control_U2 / (2 * b * l) + control_U4 / (4 * d)
        control_omega_3_square = control_U1 / (4 * b) + control_U3 / (2 * b * l) - control_U4 / (4 * d)
        control_omega_4_square = control_U1 / (4 * b) + control_U2 / (2 * b * l) + control_U4 / (4 * d)
        if control_omega_1_square > 0:
            control_omega_1 = control_omega_1_square ** 0.5
        else:
            control_omega_1 = 0
        if control_omega_2_square > 0:
            control_omega_2 = control_omega_2_square ** 0.5
        else:
            control_omega_2 = 0
        if control_omega_3_square > 0:
            control_omega_3 = control_omega_3_square ** 0.5
        else:
            control_omega_3 = 0
        if control_omega_4_square > 0:
            control_omega_4 = control_omega_4_square ** 0.5
        else:
            control_omega_4 = 0
        control_omega_1_k_1 = control_omega_1
        control_omega_2_k_1 = control_omega_2
        control_omega_3_k_1 = control_omega_3
        control_omega_4_k_1 = control_omega_4

        # Calculando tensão a ser enviada para os motores
        control_V1 = (control_omega_1 * (1 - Ap * T) - control_omega_1_k_1 - T * Cp) / (T * Bp)
        control_V2 = (control_omega_2 * (1 - Ap * T) - control_omega_2_k_1 - T * Cp) / (T * Bp)
        control_V3 = (control_omega_3 * (1 - Ap * T) - control_omega_3_k_1 - T * Cp) / (T * Bp)
        control_V4 = (control_omega_4 * (1 - Ap * T) - control_omega_4_k_1 - T * Cp) / (T * Bp)

        if control_V1 < 0:
            control_V1 = 0
        if control_V1 > v:
            control_V1 = v
        if control_V2 < 0:
            control_V2 = 0
        if control_V2 > v:
            control_V2 = v
        if control_V3 < 0:
            control_V3 = 0
        if control_V3 > v:
            control_V3 = v
        if control_V4 < 0:
            control_V4 = 0
        if control_V4 > v:
            control_V4 = v

        duty_cycle_1_vector.append(control_V1 / v * 100)
        duty_cycle_2_vector.append(control_V2 / v * 100)
        duty_cycle_3_vector.append(control_V3 / v * 100)
        duty_cycle_4_vector.append(control_V4 / v * 100)

        """Interface controlador sistema"""
        V_1 = control_V1
        V_2 = control_V2
        V_3 = control_V3
        V_4 = control_V4

        """Cálculos do sistema 
        utiliza tensão dos motores como entrada"""

        roll_k_2 = roll_k_1
        roll_k_1 = roll
        pitch_k_2 = pitch_k_1
        pitch_k_1 = pitch
        yaw_k_2 = yaw_k_1
        yaw_k_1 = yaw
        height_k_2 = height_k_1
        height_k_1 = height

        omega_1_k_1 = omega_1
        omega_2_k_1 = omega_2
        omega_3_k_1 = omega_3
        omega_4_k_1 = omega_4

        # Calculando velocidade dos motores
        omega_1 = (omega_1_k_1 + T * Bp * V_1 + T * Cp) / (1 - Ap * T)
        omega_2 = (omega_2_k_1 + T * Bp * V_2 + T * Cp) / (1 - Ap * T)
        omega_3 = (omega_3_k_1 + T * Bp * V_3 + T * Cp) / (1 - Ap * T)
        omega_4 = (omega_4_k_1 + T * Bp * V_4 + T * Cp) / (1 - Ap * T)

        # Calculando Torques
        U_1 = b * (omega_1 ** 2 + omega_2 ** 2 + omega_3 ** 2 + omega_4 ** 2)
        U_2 = -l * b * omega_2 ** 2 + l * b * omega_4 ** 2
        U_3 = -l * b * omega_1 ** 2 + l * b * omega_3 ** 2
        U_4 = d * (-(omega_1 ** 2) + omega_2 ** 2 - (omega_3 ** 2) + omega_4 ** 2)

        # Calculando estados do drone
        roll = T ** 2 * U_2 / Ixx + 2 * roll_k_1 - roll_k_2
        pitch = T ** 2 * U_3 / Iyy + 2 * pitch_k_1 - pitch_k_2
        yaw = T ** 2 * U_4 / Izz + 2 * yaw_k_1 - yaw_k_2
        height = T ** 2 * (-g + (np.cos(pitch) * np.cos(roll)) * U_1 / m) + 2 * height_k_1 - height_k_2

        # Adicionando ruídos ambientais absolutos
        height_noise = np.random.normal(0, 0.0001)
        if flag_noise:
            height += height_noise
        roll_noise = np.random.normal(0, 0.0001)
        if flag_noise:
            roll += roll_noise
        pitch_noise = np.random.normal(0, 0.0001)
        if flag_noise:
            pitch += pitch_noise
        yaw_noise = np.random.normal(0, 0.0001)
        if flag_noise:
            yaw += yaw_noise

        # Limitação física de altura
        if height < 0:
            height = 0

        """Salvando resultados"""
        height_vector.append(height)
        roll_vector.append(roll / np.pi * 180)
        pitch_vector.append(pitch / np.pi * 180)
        time_vector.append(tempo)
        tempo += T

    height_list_of_vectors.append(height_vector)
    time_list_of_vectors.append(time_vector)
    roll_list_of_vectors.append(roll_vector)
    pitch_list_of_vectors.append(pitch_vector)
    duty_cycle_1_list_of_vectors.append(duty_cycle_1_vector)
    duty_cycle_2_list_of_vectors.append(duty_cycle_2_vector)
    duty_cycle_3_list_of_vectors.append(duty_cycle_3_vector)
    duty_cycle_4_list_of_vectors.append(duty_cycle_4_vector)

"""Gráfico dos estados"""
fig, axs = plt.subplots(1, 3)
for height_vector in height_list_of_vectors:
    axs[0].plot(time_vector, height_vector)
axs[0].grid()
axs[0].set_xlabel("Tempo [s]")
axs[0].set_ylabel("Altura [m]")
for roll_vector in roll_list_of_vectors:
    axs[1].plot(time_vector, roll_vector)
axs[1].grid()
axs[1].set_xlabel("Tempo [s]")
axs[1].set_ylabel("Rolagem [°]")
axs[1].set_ylim([-10, 10])
for pitch_vector in pitch_list_of_vectors:
    axs[2].plot(time_vector, pitch_vector)
axs[2].grid()
axs[2].set_xlabel("Tempo [s]")
axs[2].set_ylabel("Arfagem [°]")
axs[2].set_ylim([-10, 10])
fig.set_size_inches(10, 4, forward=True)
plt.tight_layout()
plt.show()

"""Gráfico das ações de controle"""
fig, axs = plt.subplots(2, 2)
for duty_cycle_1_vector in duty_cycle_1_list_of_vectors:
    axs[0, 0].plot(time_vector, duty_cycle_1_vector)
axs[0, 0].grid()
axs[0, 0].set_xlabel("Tempo [s]")
axs[0, 0].set_ylabel("Duty cycle motor 1 [%]")
for duty_cycle_2_vector in duty_cycle_2_list_of_vectors:
    axs[0, 1].plot(time_vector, duty_cycle_2_vector)
axs[0, 1].grid()
axs[0, 1].set_xlabel("Tempo [s]")
axs[0, 1].set_ylabel("Duty cycle motor 2 [%]")
for duty_cycle_3_vector in duty_cycle_3_list_of_vectors:
    axs[1, 0].plot(time_vector, duty_cycle_3_vector)
axs[1, 0].grid()
axs[1, 0].set_xlabel("Tempo [s]")
axs[1, 0].set_ylabel("Duty cycle motor 3 [%]")
for duty_cycle_4_vector in duty_cycle_4_list_of_vectors:
    axs[1, 1].plot(time_vector, duty_cycle_4_vector)
axs[1, 1].grid()
axs[1, 1].set_xlabel("Tempo [s]")
axs[1, 1].set_ylabel("Duty cycle motor 4 [%]")
fig.set_size_inches(6, 4, forward=True)
plt.tight_layout()
plt.show()
