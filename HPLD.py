# -*- coding: utf-8 -*-
import collections
import copy

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
    tasks_array = np.array(tasks)
    return tasks_array

# 得到连接请求的最短路径, 并按照降序进行排列
def Paths_order(tasks, G):
    """
    :param   tasks: 任务请求
    :param       G: 图的结构信息
    :return: order: 任务连接请求按照最短路径的降序排列
             Paths: 每个任务连接请求的对应最短路径
    """
    num = []      # 存储每个连接请求的最短路径数值
    Paths = []    # 存储每个连接请求的最短路径
    task_num = len(tasks) # 任务请求数量
    for i in range(task_num):
        task = tasks[i]
        exist_path, path = has_path(G, task)
        if exist_path:
          num.append(len(path)-1)
          Paths.append(path)
        else:  #找不到最短路径
            num.append(0)
            Paths.append([0])
    order = sorted(range(len(num)), key=lambda k: num[k], reverse=True)  #每个连接请求的最短路径按长度降序排列, 存储对应的任务请求序号
    Path = copy.deepcopy(Paths)
    for i in range(len(order)):
        Path[i] = Paths[order[i]]

    return order, Path

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

def has_weight_path(G, task):
    """
       :param G:     图
       :param task: 任务请求
       :return:  exist: 判断是否存在路径
    """

    distance = nx.dijkstra_path_length(G, source=task[0], target=task[1], weight='weight')
    if distance < INF_val:
        return True
    else:
        return False

def LFFP(tasks, edges, nodes):
    """
    :param          tasks: 任务请求序列
    :param          edges: 边序列
    :param          nodes: 结点序列
    :return:       Graphs: 每个波长层中剩余图的结构信息
             Occupathions: 每个波长层中边的占用情况
                LightPath: 存储每条路径所及其所对应的波长
                    order: 任务请求的降序排列顺序
                  require: 存储每个波长层中的连接请求
    """
    can_route = True  # 标识路径在该波长层中是否存在
    w = 0             # 标识波长层序号
    LightPath = []    # 存储每个连接请求的路径及其对应的波长层
    Occupathions = [] # 存储每个波长层中边的占用情况
    occupathion  = {}
    for e in edges[:]:
        occupathion[e] = False
    Occupathions.append(occupathion.copy())
    Graphs = []
    Graph = nx.DiGraph()
    for i in range(len(nodes)):
        Graph.add_node(nodes[i])
    for x, y in edges:  # edges:
        Graph.add_edges_from([(x, y)])
        Graph.add_edges_from([(y, x)])
    Graphs.append(Graph.copy())
    require = [[]]
    order, Path = Paths_order(tasks, Graph)
    for i in range(len(Path)):
        path = Path[i]
        Edge = [] # 存储路径中所设计的边
        for apk in range(len(path) - 1):
            if path[apk] < path[apk + 1]:
                edge = (path[apk], path[apk + 1])
            else:
                edge = (path[apk + 1], path[apk])
            Edge.append(edge)
        for w in range(len(Occupathions)):
            can_route = True
            # 判断该连接请求的最短路径是否存在当前的波长层中
            for e in Edge:
                if Occupathions[w][e]:
                    can_route = False
                    break
            if can_route == True: # 分配波长
                LightPath.append((path, w)) # 记录每条路径对应的波长层
                require[w].append(order[i]) # 记录每个波长层所分配的任务请求
                for e in Edge:
                   Occupathions[w][e] = True
                   Graphs[w].remove_edge(e[0], e[1])
                   Graphs[w].remove_edge(e[1], e[0])
                break
        if can_route == False: # 如果当前所有波长层都无法路由，就添加新的波长层
            Occupathions.append(occupathion.copy())
            Graphs.append(Graph.copy())
            LightPath.append((path, w+1))
            require.append([order[i]])
            for e in Edge:
                Occupathions[-1][e] = True
                Graphs[-1].remove_edge(e[0], e[1])
                Graphs[-1].remove_edge(e[1], e[0])

    return Graphs, Occupathions, LightPath, order, require


def HPLD(tasks, edges, nodes):
    """
        :param          tasks: 任务请求序列
        :param          edges: 边序列
        :param          nodes: 结点序列
        :return: Occupathions: 每个波长层中边的占用情况
                    LightPath: 存储每条路径所及其所对应的波长
                      require: 存储每个波长层中的连接请求
    """
    Graphs, Occupathions, LightPath, order, require = LFFP(tasks, edges, nodes) #利用LFFP分配路径
    task_num = len(tasks)  # 任务数量
    edge_num = len(edges)  # 边数量
    reduce_wave = []       # 存储被简化的波长层
    sigma    = 1
    edges_cost = {}
    edge_sets = []
    for i in range(task_num):
        path = LightPath[i][0]
        for apk in range(len(path) - 1):
            if path[apk] < path[apk + 1]:
                edge = (path[apk], path[apk + 1])
            else:
                edge = (path[apk + 1], path[apk])
            edge_sets.append(edge)
    edge_load = collections.Counter(edge_sets)  # 记录图中边的负载数
    while True:
        # 负载偏差量的确定
        mean_load = 0
        max_load = 0
        for e in edge_load:
            mean_load += edge_load[e]
            if max_load < edge_load[e]:
                max_load = edge_load[e]
        mean_load /= edge_num
        deta = int(sigma * (max_load - mean_load))
        if deta == 0:
            return Occupathions, LightPath, require, reduce_wave
        for e in edges:
            if max_load - 1 - edge_load[e] > 0:
                edges_cost[e] = 1 / (max_load - 1 - edge_load[e])
            else:
                edges_cost[e] = INF_val
        Max_Load_Path_index = [] #记录最大负载路径的索引
        for i in range(task_num):
            path = LightPath[i][0]
            for apk in range(len(path) - 1):
                if path[apk] < path[apk + 1]:
                    e = (path[apk], path[apk + 1])
                else:
                    e = (path[apk + 1], path[apk])
                if edge_load[e] == max_load:
                    Max_Load_Path_index.append(i)
                    break
        Graph = nx.DiGraph()
        for i in range(len(nodes)):
            Graph.add_node(nodes[i])
        for e in edges:  # edges:
            x, y = e[0], e[1]
            Graph.add_weighted_edges_from([(x, y, edges_cost[e])])
            Graph.add_weighted_edges_from([(y, x, edges_cost[e])])

        Exist_Path = []
        for path_idx in Max_Load_Path_index:
            exist_path = has_weight_path(Graph, tasks[order[path_idx]])
            if exist_path:
                Exist_Path.append(path_idx)
        if not Exist_Path:
            return Occupathions, LightPath, require, reduce_wave
        else:
            not_change = True
            while not_change:
              if not Exist_Path:
                 break
              random_index = random.choice(Exist_Path)
              Exist_Path.remove(random_index)
              p = LightPath[random_index][0]  # 得到原路径
              ori = LightPath[random_index][1]  # 得到原路径所在波长层
              ori_edges = [] # 记录原路径的边
              #移除原路径
              for apk in range(len(p) - 1):
                 if p[apk] < p[apk + 1]:
                    e = (p[apk], p[apk + 1])
                 else:
                    e = (p[apk + 1], p[apk])
                 ori_edges.append(e)
                 edge_load[e] -= 1
                 Occupathions[ori][e] = False
                 Graphs[ori].add_edges_from([(e[0], e[1])])
                 Graphs[ori].add_edges_from([(e[1], e[0])])
                 if len(Graphs[ori].edges()) == 2*edge_num:
                    reduce_wave.append(ori)
              #计算新路径
              path = nx.dijkstra_path(Graph, source=tasks[order[random_index]][0], target=tasks[order[random_index]][1])
              Edge = []            # 存储新的路径
              for apk in range(len(path) - 1):
                if path[apk] < path[apk + 1]:
                    edge = (path[apk], path[apk + 1])
                else:
                    edge = (path[apk + 1], path[apk])
                Edge.append(edge)
              can_route = True
              for w in range(len(Occupathions)):
                 if w in reduce_wave:
                    continue
                 can_route = True
                 for e in Edge:
                    if Occupathions[w][e]:
                        can_route = False
                        break
                 if can_route:
                    require[ori].remove(order[random_index])
                    require[w].append(order[random_index])
                    LightPath[random_index] = (path, w)
                    for e in Edge:
                        Occupathions[w][e] = True
                        Graphs[w].remove_edge(e[0], e[1])
                        Graphs[w].remove_edge(e[1], e[0])
                        edge_load[e] += 1
                        if e not in ori_edges:  # 判断新边是否出现在旧路径里
                            not_change = False
                    for e in ori_edges:
                        if e not in Edge:  # 判断旧边是否出现在新路径里
                            not_change = False
                    break
              if can_route == False:
                if ori in reduce_wave:
                    reduce_wave.remove(ori)
                for apk in range(len(p) - 1):
                    if p[apk] < p[apk + 1]:
                        e = (p[apk], p[apk + 1])
                    else:
                        e = (p[apk + 1], p[apk])
                    Occupathions[ori][e] = True
                    Graphs[ori].remove_edge(e[0], e[1])
                    Graphs[ori].remove_edge(e[1], e[0])
            if not_change:
                return Occupathions, LightPath, require, reduce_wave






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

        Occupathions, LightPath, require, reduce_wave = HPLD(tasks, edges, nodes)

        end_time = time.time()  # 求解终止时间
        # print(end_time, start_time)
        times.append(end_time - start_time)
        ns = 0
        for i in range(len(LightPath)):
            ns = len(LightPath[i][0]) + ns - 1
        wave_link_length.append(ns)

        wave_num.append(len(Occupathions) - len(reduce_wave))
    print(wave_link_length)
    print(sum(wave_link_length))
    print(sum(times))
    print(wave_num)
    print(sum(wave_num))