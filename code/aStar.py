import numpy as np
import math
import matplotlib.pyplot as plt


def adjacent(m, pos):
    """
    Computes the accessible neighbors assuming a 4-grid
    for a given map and position
    :param m: Matrix-encoded environment (0 free space; 1 obstacle)
    :param pos: Position (row, column)
    :return: List of neighbors [(row1, column1), (row2, column2), ...]
    """
    result = []
    if pos[0] > 0 and m[pos[0] - 1, pos[1]] == 0:
        result.append((pos[0] - 1, pos[1]))
    if pos[0] < m.shape[0] - 1 and m[pos[0] + 1, pos[1]] == 0:
        result.append((pos[0] + 1, pos[1]))
    if pos[1] > 0 and m[pos[0], pos[1] - 1] == 0:
        result.append((pos[0], pos[1] - 1))
    if pos[1] < m.shape[1] - 1 and m[pos[0], pos[1] + 1] == 0:
        result.append((pos[0], pos[1] + 1))
    return result


def heuristic(a, b):
    """
    Computes the Manhatten distance between two points
    :param a: start position (row, column)
    :param b: goal position (row, column)
    :return: manhatten distance between a and b
    """
    return math.fabs(a[0] - b[0]) + math.fabs(a[1] - b[1])


def valid(m, pos):
    """
    Returns if the given position is free space or not.
    :param m: Matrix-encoded environment (0 free space; 1 obstacle)
    :param pos: position to check (row, column)
    :return: True, if position is in free space
    """
    return m.shape[0] > pos[0] >= 0 and m.shape[1] > pos[1] >= 0 and m[pos] == 0


def a_star(m, start_pos, goal_pos):
    """
    Computes the shortest path from start_pos to goal_pos using A*
    :param m: Matrix-encoded environment (0 free space; 1 obstacle)
    :param start_pos: start position (row, column)
    :param goal_pos: goal position (row, column)
    :return: Path (list of row/column tuples)
    """

    print("in aStar class, m[6][3] is {}".format(m[6][3]))
    if not valid(m, start_pos) or not valid(m, goal_pos):
        return None

    open_set = set()
    closed_set = set()
    f_score = dict()
    g_score = dict()
    parent = dict()

    f_score[start_pos] = heuristic(start_pos, goal_pos)
    g_score[start_pos] = 0

    open_set.add(start_pos)
    n = None
    while len(open_set) > 0:
        # find item with lowest f-score
        n = min(open_set, key=lambda x: f_score[x])
        if n == goal_pos:
            break
        open_set.remove(n)
        closed_set.add(n)
        for a in adjacent(m, n):
            if a in closed_set:
                continue
            tentative_g_score = g_score[n] + 1
            if a not in open_set or tentative_g_score < g_score[a]:
                parent[a] = n
                g_score[a] = tentative_g_score
                f_score[a] = g_score[a] + heuristic(a, goal_pos)
                if a not in open_set:
                    open_set.add(a)

    if n != goal_pos:
        return None

    # reconstruct path:
    p = goal_pos
    path = [p]
    while p != start_pos:
        p = parent[p]
        path.append(p)
    path.reverse()
    return path


if __name__ == "__main__":
    m = np.loadtxt("map14.txt", delimiter=",")

    path = a_star(m, (9, 1), (3, 9))
    print(path)
    if path:
        for p in path:
            m[p] = 2
        m[m == 0] = 255
        m[m == 1] = 0
        m[m == 2] = 128
        plt.matshow(m, cmap=plt.cm.gray)
        plt.show()
    else:
        print("No solution found!")
