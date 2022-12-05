import numpy as np
from matplotlib import pyplot as plt

def calculate_average(graph_name:str, cutoff:int, opt:int):
    '''
    calculate average performance of random seeds 0 to 9
    example: calculate_average('power', 10, 2203)
    :param graph_name:
    :param cutoff:
    :param opt:
    :return:
    '''
    rt = []
    v = []
    for i in range(10):
        file_name = f'{graph_name}_LS1_{cutoff}_{i}.trace'
        with open(file_name, 'r') as file:
            last_line = file.readlines()[-1]
            values = last_line.split(', ')
            run_time, vc = values[0], values[1]
            run_time = float(run_time)
            vc = int(vc)
            rt.append(run_time)
            v.append(vc)
    print(np.average(rt))
    avg_v = np.average(v)
    rel = (avg_v - opt)/opt
    print(np.average(v))
    print(rel)


def QRTD(graph_name: str, cutoff:int, q:float, opt:int):
    '''
    example:
    QRTD('power', 10, 0.45, 2203)
    QRTD('power', 10, 0.50, 2203)
    QRTD('power', 10, 0.55, 2203)
    QRTD('power', 10, 0.60, 2203)
    plt.show()
    :param graph_name:
    :param cutoff:
    :param q:
    :param opt:
    :return:
    '''
    time_to_reach = np.ones(100)*np.inf
    perf = opt*(1+q)
    for i in range(100):
        file_name = f'{graph_name}_LS1_{cutoff}_{i}.trace'
        with open(file_name, 'r') as file:
            for line in file:
                values = line.split(', ')
                run_time, vc = values[0], values[1]
                run_time = float(run_time)
                vc = int(vc)
                if vc <= perf:
                    time_to_reach[i] = run_time
                    break
    time_to_reach = np.sort(time_to_reach)
    curr = 0
    x = []
    y = []
    for t in time_to_reach:
        if t == np.inf:
            break
        x.append(t)
        curr += 0.01
        y.append(curr)
    plt.xlabel('time')
    plt.ylabel('proportion')
    plt.plot(x, y, label=f'q*={q}')
    plt.legend()


def SQD(graph_name: str, cutoff:int, t:float, opt:int):
    '''
    example:
    SQD('star2', 10, 2, 4542)
    SQD('star2', 10, 4, 4542)
    SQD('star2', 10, 6, 4542)
    SQD('star2', 10, 8, 4542)
    :param graph_name:
    :param cutoff:
    :param t:
    :param opt:
    :return:
    '''
    quality = np.inf*np.ones(100)
    for i in range(100):
        file_name = f'{graph_name}_LS1_{cutoff}_{i}.trace'
        with open(file_name, 'r') as file:
            for line in file:
                values = line.split(', ')
                run_time, vc = values[0], values[1]
                run_time = float(run_time)
                vc = int(vc)
                if run_time > t:
                    break
                if vc < quality[i]:
                    quality[i] = vc
    quality = np.sort(quality)
    quality = (quality - opt)/opt
    y = np.linspace(0, 1, 101)[1:]
    plt.xlabel('quality(relative error)')
    plt.ylabel('proportion')
    plt.plot(quality, y, label=f't={t}')
    plt.legend()


def box_plot(graph_name: str, cutoff:int):
    rt = []
    for i in range(100):
        file_name = f'{graph_name}_LS1_{cutoff}_{i}.trace'
        with open(file_name, 'r') as file:
            last_line = file.readlines()[-1]
            values = last_line.split(', ')
            run_time, vc = values[0], values[1]
            run_time = float(run_time)
            vc = int(vc)
            rt.append(run_time)
    plt.boxplot(rt)
    plt.show()


box_plot('star2', 10)