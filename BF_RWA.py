# -*- coding: utf-8 -*-
import numpy as np
import networkx as nx
import pickle as cp
import random
import ctypes
import os
import sys
import time
import glob
import re
import math
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

## 表示无穷大
INF_val = float("inf")

# 生成任务
def generator_task(task_num, node_nums):
    """
    :param task_num  : 需要生成的任务数
    :param node_nums : 图中结点数量
    :return: tasks   : 生成的任务连接请求
    """
    tasks = []
    for _ in range(task_num):
        task = random.sample(range(node_nums), 2)
        tasks.append((task[0], task[1]))
    return tasks

#判断任务请求在图G中是否存在路径, 若存在返回对应路径
def has_path(G, task):
    """
    :param G:     图
    :param task: 任务请求
    :return:  path:路径
    """
    try:

        path = nx.shortest_path(G, source=task[0], target=task[1])

    except nx.NetworkXNoPath:
        return False, None
    return True, path


#连接请求按照随机顺序，依次从已使用的波长图中寻找最短路径进行路由
def BF_RWA(tasks, edges, nodes):
    """
       :param tasks: 任务连接请求
       :param edges: 边序列
       :param nodes: 结点序列
       :return:   G: 每一个波长层的剩余边连接关系
          LightPath: 存储每条路径所及其所对应的波长
    """
    order = np.arange(len(tasks))
    np.random.shuffle(order)  # 随机打乱任务请求的顺序
    LightPath = []  # 存储每个连接请求的路径及其对应的波长层
    Graph = nx.DiGraph()
    for i in range(len(nodes)):
        Graph.add_node(nodes[i])
    for x, y in edges:  # edges::
        Graph.add_edges_from([(x, y)])
        Graph.add_edges_from([(y, x)])
    Graphs = []
    Graphs.append(Graph.copy())
    for id in order:
        task = tasks[id]
        val = INF_val
        wave_length = -1
        lightpath   = None
        for w in range(len(Graphs)):
            G = Graphs[w]
            exist_path, path = has_path(G, task)
            if exist_path:
                if len(path) - 1 < val:
                    val = len(path) - 1
                    wave_length = w
                    lightpath   = path
        if wave_length == -1:
            Graphs.append(Graph.copy())
            G = Graphs[-1]
            path = nx.shortest_path(G, source=task[0], target=task[1])
            LightPath.append((path, len(Graphs) - 1))
            for apk in range(len(path) - 1):
                e = (path[apk], path[apk + 1])
                Graphs[-1].remove_edge(e[0], e[1])
                Graphs[-1].remove_edge(e[1], e[0])
        else:
            LightPath.append((lightpath, wave_length))
            for apk in range(len(lightpath) - 1):
                e = (lightpath[apk + 1], lightpath[apk])
                Graphs[wave_length].remove_edge(e[0], e[1])
                Graphs[wave_length].remove_edge(e[1], e[0])
    return Graphs, LightPath

if __name__ == "__main__":
    random.seed(2)  # 设置随机种子
    wave_link_length = []  # 波长链路数
    times = []  # 时间
    wave_num = []  # 波长数
    node_nums = 14  # 结点数量
    nodes = np.arange(node_nums)  # 结点序列
    edges = [(0, 1), (0, 2), (0, 7), (1, 2), (1, 3), (2, 5), (3, 4), (3, 10), (4, 5), (4, 6), (5, 9), (5, 13),
             (6, 7), (7, 8), (8, 9), (8, 11), (8, 12), (10, 11), (10, 13), (11, 13), (12, 13)]
    # 边连接关系
    Max_num_tasks = 200
    for task_num in range(1, Max_num_tasks + 1):

        tasks = generator_task(task_num, node_nums)  # 生成任务

        start_time = time.time()  # 求解起始时间

        G, LightPath = BF_RWA(tasks, edges, nodes)

        end_time = time.time()  # 求解终止时间

        times.append(end_time - start_time)
        ns = 0
        for i in range(len(LightPath)):
            ns = len(LightPath[i][0]) + ns - 1
        wave_link_length.append(ns)

        wave_num.append(len(G))
    print(wave_link_length)
    print(sum(wave_link_length))
    print(sum(times))
    print(wave_num)
    print(sum(wave_num))
