#%%
#! /usr/bin/env python3

import numpy as np
from transform_utils import *



def make_decoupled_trajectory(current_matrix, final_matrix):
    traj = []
    pos_init = np.array([current_matrix[0, 3], current_matrix[1, 3], current_matrix[2, 3]])
    pos_final = np.array([final_matrix[0, 3], final_matrix[1, 3], final_matrix[2, 3]])
    static_rot_matrix = current_matrix[3:, 3:]
    dist = np.sqrt(np.sum(np.square(np.abs(pos_final - pos_init))))
    time = 10 * dist
    ps = []
    for i in range(int(100 * time)):
        s = (i + 1) / (time * 100)
        ps = pos_init + s * (pos_final - pos_init)
        traj.append([static_rot_matrix[0, 0], static_rot_matrix[0, 1], static_rot_matrix[0, 2],
                     static_rot_matrix[1, 0], static_rot_matrix[1, 1], static_rot_matrix[1, 2],
                     static_rot_matrix[2, 0], static_rot_matrix[2, 1], static_rot_matrix[2, 2],
                                       ps[0],                   ps[1],                  ps[2]])
        current_matrix[0, 3], reached_matrix[1, 3], reached_matrix[2, 3] = ps[0], ps[1], ps[2]
        return traj, current_matrix


def make_decoupled_3_trajectory(current_matrix, final_matrix):
    traj = []
    pos_init = np.array([current_matrix[0, 3], current_matrix[1, 3], current_matrix[2, 3]])
    pos_final = np.array([final_matrix[0, 3], final_matrix[1, 3], final_matrix[2, 3]])
    static_rot_matrix = current_matrix[3:, 3:]
    dist = np.sqrt(np.sum(np.square(np.abs(pos_final - pos_init))))
    time = 10 * dist
    ps = []
    for i in range(int(100 * time)):
        T = int(100 * time)
        a2, a3 = 3 / (T ** 3), -2 / (T ** 3)
        s = a2 * (i + 1) ** 2 + a3 * (i + 1) ** 3
        ps = pos_init + s * (pos_final - pos_init)
        traj.append([static_rot_matrix[0, 0], static_rot_matrix[0, 1], static_rot_matrix[0, 2],
                     static_rot_matrix[1, 0], static_rot_matrix[1, 1], static_rot_matrix[1, 2],
                     static_rot_matrix[2, 0], static_rot_matrix[2, 1], static_rot_matrix[2, 2],
                                       ps[0],                   ps[1],                  ps[2]])
        current_matrix[0, 3], reached_matrix[1, 3], reached_matrix[2, 3] = ps[0], ps[1], ps[2]
        return traj, current_matrix


def make_decoupled_5_trajectory(current_matrix, final_matrix):
    traj = []
    pos_init = np.array([current_matrix[0, 3], current_matrix[1, 3], current_matrix[2, 3]])
    pos_final = np.array([final_matrix[0, 3], final_matrix[1, 3], final_matrix[2, 3]])
    static_rot_matrix = current_matrix[3:, 3:]
    dist = np.sqrt(np.sum(np.square(np.abs(pos_final - pos_init))))
    time = 10 * dist
    ps = []
    for i in range(int(100 * time)):
        T = int(100 * time)
        a3, a4, a5 = 10 / (T ** 3), -15 / (T ** 4), 6 / (T ** 5)
        s = a3 * (i + 1) ** 3 + a4 * (i + 1) ** 4 + a5 * (i + 1) ** 5
        ps = pos_init + s * (pos_final - pos_init)
        traj.append([static_rot_matrix[0, 0], static_rot_matrix[0, 1], static_rot_matrix[0, 2],
                     static_rot_matrix[1, 0], static_rot_matrix[1, 1], static_rot_matrix[1, 2],
                     static_rot_matrix[2, 0], static_rot_matrix[2, 1], static_rot_matrix[2, 2],
                                       ps[0],                   ps[1],                  ps[2]])
        current_matrix[0, 3], reached_matrix[1, 3], reached_matrix[2, 3] = ps[0], ps[1], ps[2]
        return traj, current_matrix


def make_screw_trajectory(current_matrix, final_matrix):
    traj = []
    pos_init = np.array([current_matrix[0, 3], current_matrix[1, 3], current_matrix[2, 3]])
    pos_final = np.array([final_matrix[0, 3], final_matrix[1, 3], final_matrix[2, 3]])
    dist = np.sqrt(np.sum(np.square(np.abs(pos_final - pos_init))))
    time = 10 * dist
    for i in range(int(100 * time)):
        s = (i + 1) / int(100 * time)
        Ts = current_matrix @ MatrixExp6(MatrixLog6(TransInv(current_matrix) @ final_matrix) * s)
        traj.append([Ts[0, 0], Ts[0, 1], Ts[0, 2],
                     Ts[1, 0], Ts[1, 1], Ts[1, 2],
                     Ts[2, 0], Ts[2, 1], Ts[2, 2],
                     Ts[0, 3], Ts[1, 3], Ts[2, 3]])
    current_matrix = Ts
    return traj, current_matrix


def make_screw_Poly3_trajectory(current_matrix, final_matrix):
    traj = []
    pos_init = np.array([current_matrix[0, 3], current_matrix[1, 3], current_matrix[2, 3]])
    pos_final = np.array([final_matrix[0, 3], final_matrix[1, 3], final_matrix[2, 3]])
    dist = np.sqrt(np.sum(np.square(np.abs(pos_final - pos_init))))
    time = 10 * dist
    for i in range(int(100 * time)):
        T = int(100 * time)
        a2, a3 = 3 / (T ** 3), -2 / (T ** 3)
        s = a2 * (i + 1) ** 2 + a3 * (i + 1) ** 3
        Ts = current_matrix @ MatrixExp6(MatrixLog6(TransInv(current_matrix) @ final_matrix) * s)
        traj.append([Ts[0, 0], Ts[0, 1], Ts[0, 2],
                     Ts[1, 0], Ts[1, 1], Ts[1, 2],
                     Ts[2, 0], Ts[2, 1], Ts[2, 2],
                     Ts[0, 3], Ts[1, 3], Ts[2, 3]])   
    current_matrix = Ts
    return traj, current_matrix


def make_screw_Poly5_trajectory(current_matrix, final_matrix):
    traj = []
    pos_init = np.array([current_matrix[0, 3], current_matrix[1, 3], current_matrix[2, 3]])
    pos_final = np.array([final_matrix[0, 3], final_matrix[1, 3], final_matrix[2, 3]])
    dist = np.sqrt(np.sum(np.square(np.abs(pos_final - pos_init))))
    time = 10 * dist
    for i in range(int(100 * time)):
        T = int(100 * time)
        a3, a4, a5 = 10 / (T ** 3), -15 / (T ** 4), 6 / (T ** 5)
        s = a3 * (i + 1) ** 3 + a4 * (i + 1) ** 4 + a5 * (i + 1) ** 5
        Ts = current_matrix @ MatrixExp6(MatrixLog6(TransInv(current_matrix) @ final_matrix) * s)
        traj.append([Ts[0, 0], Ts[0, 1], Ts[0, 2],
                     Ts[1, 0], Ts[1, 1], Ts[1, 2],
                     Ts[2, 0], Ts[2, 1], Ts[2, 2],
                     Ts[0, 3], Ts[1, 3], Ts[2, 3]])
    current_matrix = Ts
    return traj, current_matrix
