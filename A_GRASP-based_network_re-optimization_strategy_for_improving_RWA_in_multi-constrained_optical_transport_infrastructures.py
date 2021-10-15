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


class Dijkstra_Path():
    def __init__(self, node_map):
        self.node_map = node_map
        self.node_length = len(node_map)
        self.used_node_list = []
        self.collected_node_dict = {}

    def __call__(self, from_node, to_node):
        self.from_node = from_node
        self.to_node = to_node
        self._init_dijkstra()
        return self._format_path()

    def _init_dijkstra(self):
        ## Add from_node to used_node_list
        self.used_node_list.append(self.from_node)
        for index1 in range(self.node_length):
            self.collected_node_dict[index1] = [INF_val, -1]

        self.collected_node_dict[self.from_node] = [0, -1]  # from_node don't have pre_node
        for index1, weight_val in enumerate(self.node_map[self.from_node]):
            if weight_val:
                self.collected_node_dict[index1] = [weight_val, self.from_node]  # [weight_val, pre_node]
        self._foreach_dijkstra()

    def _foreach_dijkstra(self):
        while (len(self.used_node_list) < self.node_length - 1):
            min_key = -1
            min_val = INF_val
            for key, val in self.collected_node_dict.items():  # 遍历已有权值节点
                if val[0] < min_val and key not in self.used_node_list:
                    min_key = key
                    min_val = val[0]

                    ## 把最小的值加入到used_node_list
            if min_key != -1:
                self.used_node_list.append(min_key)
            else:
                return [(self.from_node, INF_val)]

            for index1, weight_val in enumerate(self.node_map[min_key]):
                ## 对刚加入到used_node_list中的节点的相邻点进行遍历比较
                if weight_val > 0 and self.collected_node_dict[index1][0] > weight_val + min_val:
                    self.collected_node_dict[index1][0] = weight_val + min_val  # update weight_val
                    self.collected_node_dict[index1][1] = min_key

    def _format_path(self):
        node_list = []
        temp_node = self.to_node
        node_list.append((temp_node, self.collected_node_dict[temp_node][0]))
        while self.collected_node_dict[temp_node][1] != -1:
            temp_node = self.collected_node_dict[temp_node][1]
            node_list.append((temp_node, self.collected_node_dict[temp_node][0]))
        node_list.reverse()
        return node_list


def set_node_map(node_map, node, node_list):
    for x, y in node_list:
        node_map[node.index(x)][node.index(y)] = node_map[node.index(y)][node.index(x)] = 1


def set_node_map_1(node_map, node, node_list):
    for x, y, z in node_list:
        node_map[node.index(x)][node.index(y)] = node_map[node.index(y)][node.index(x)] = z


def LEEP(order, Path, edges):
    global can_route, w
    LightPath = []
    W = [edges[:]]
    require = [[]]
    for path in range(len(Path)):
        Edge = []
        for apk in range(len(Path[path]) - 1):
            if Path[path][apk][0] < Path[path][apk + 1][0]:
                edge = (Path[path][apk][0], Path[path][apk + 1][0])
            else:
                edge = (Path[path][apk + 1][0], Path[path][apk][0])
            Edge.append(edge)
        for w in range(len(W)):
            can_route = True
            for e in Edge:
                if e not in W[w]:
                    can_route = False
                    break
            if can_route == True:
                LightPath.append(Path[path])
                LightPath.append(w)
                require[w].append(order[path])
                for e in Edge:
                    W[w].remove(e)
                break
        if can_route == False:
            w_edges = edges[:]
            LightPath.append(Path[path])
            LightPath.append(w + 1)
            require.append([order[path]])
            for e in Edge:
                w_edges.remove(e)
            W.append(w_edges)

    return W, LightPath, require


def shunxu(K, edges, nodes):
    node_map = [[0 for v in range(len(nodes))] for v in range(len(nodes))]
    set_node_map(node_map, nodes, edges)
    num = []
    Paths = []
    for i in K:
        dijkstrapath = Dijkstra_Path(node_map)
        path = dijkstrapath(i[0], i[1])
        num.append(path[-1][1])
        Paths.append(path)
    order = sorted(num)[::-1]
    Path = copy.deepcopy((Paths))
    for i in range(len(order)):
        order[i] = num.index(order[i])
        Path[i] = Paths[order[i]]
        num[order[i]] = 0

    return order, Path


def ls(LightPath, edges, W, require, order):
    for i in range(len(edges)):
        edges[i] = (edges[i][0], edges[i][1], 1)
    for i in range(1):
        for id in range(len(order)):
            ag = []
            for path in range(len(LightPath) // 2):
                if path == id:
                    continue
                for apk in range(len(LightPath[2 * path]) - 1):
                    if LightPath[2 * path][apk][0] < LightPath[2 * path][apk + 1][0]:
                        edge = (LightPath[2 * path][apk][0], LightPath[2 * path][apk + 1][0])
                    else:
                        edge = (LightPath[2 * path][apk + 1][0], LightPath[2 * path][apk][0])
                    ag.append(edge)
            al = collections.Counter(ag)
            p_max = 0
            for edge in al:
                if p_max < al[edge]:
                    p_max = al[edge]
            for i in range(len(edges)):
                edges[i] = (edges[i][0], edges[i][1], al[(edges[i][0], edges[i][1])])
            node_map = [[0 for v in range(len(nodes))] for v in range(len(nodes))]
            set_node_map_1(node_map, nodes, edges)
            dijkstrapath = Dijkstra_Path(node_map)
            path = dijkstrapath(K[order[id]][0], K[order[id]][1])
            if path[-1][1] == INF_val:
                continue
            for apk in range(len(LightPath[2 * id]) - 1):
                if LightPath[2 * id][apk][0] < LightPath[2 * id][apk + 1][0]:
                    edge = (LightPath[2 * id][apk][0], LightPath[2 * id][apk + 1][0])
                else:
                    edge = (LightPath[2 * id][apk + 1][0], LightPath[2 * id][apk][0])
                W[LightPath[2 * id + 1]].append(edge)
            Edge = []
            for apk in range(len(path) - 1):
                if path[apk][0] < path[apk + 1][0]:
                    edge = (path[apk][0], path[apk + 1][0])
                else:
                    edge = (path[apk + 1][0], path[apk][0])
                Edge.append(edge)
            for w in range(len(W)):
                can_route = True
                for e in Edge:
                    if e not in W[w]:
                        can_route = False
                        break
                if can_route == True:
                    require[LightPath[2 * id + 1]].remove(order[id])
                    require[w].append(order[id])
                    LightPath[2 * id] = path
                    LightPath[2 * id + 1] = w
                    for e in Edge:
                        W[w].remove(e)
                    break
            if not can_route:
                for apk in range(len(LightPath[2 * id]) - 1):
                    if LightPath[2 * id][apk][0] < LightPath[2 * id][apk + 1][0]:
                        edge = (LightPath[2 * id][apk][0], LightPath[2 * id][apk + 1][0])
                    else:
                        edge = (LightPath[2 * id][apk + 1][0], LightPath[2 * id][apk][0])
                    W[LightPath[2 * id + 1]].remove(edge)

    return W, LightPath, require


random.seed(2)
if __name__ == "__main__":
    a = []  # 波长链路
    b = []  # 时间
    c = []  # 波长

    for inj in range(200):

        edges = [(0, 1), (0, 2), (0, 7), (1, 2), (1, 3), (2, 5), (3, 4), (3, 10), (4, 5), (4, 6), (5, 9), (5, 13),
                 (6, 7), (7, 8), (8, 9), (8, 11), (8, 12), (10, 11), (10, 13), (11, 13), (12, 13)]
        nodes = range(14)
        renwushu = inj
        K = []
        for vad in range(renwushu):
            res = random.sample(range(len(nodes)), 2)
            K.append((res[0], res[1]))

        time1 = time.time()

        order, Path = shunxu(K, edges, nodes)

        W, LightPath, require = LEEP(order, Path, edges)

        W, LightPath, require = ls(LightPath, edges, W, require, order)

        time2 = time.time()
        print(time2, time1)
        b.append(time2 - time1)

        ns = 0
        for i in range(len(LightPath) // 2):
            ns = len(LightPath[2 * i]) + ns - 1
        a.append(ns)

        an = 0
        for i in W:
            if len(i) < len(edges):
                an += 1
            elif len(i) > len(edges):
                an = INF_val
        print(an)
        c.append(an)
    print(a)
    print(b)
    print(c)

