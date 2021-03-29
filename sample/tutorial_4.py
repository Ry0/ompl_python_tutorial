import ompl
from ompl import base as ob
from ompl import geometric as og
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

SPACE_DIM = 2

obstacle = {((-0.85, -0.15), (-0.45, 0.50)),
            ((-0.30, -0.30), (0.30, 0.30)),
            ((0.30, 0.00), (0.60, 0.60)),
            ((0.00, -0.75), (0.50, -0.50))}
start_s = (-0.80, 0.80)
goal_s = (0.80, -0.80)


def isStateValid(state):
    # 指定された矩形に含まれているかを確認。
    for o in obstacle:
        if (o[0][0] <= state[0] and state[0] <= o[1][0]) and (o[0][1] <= state[1] and state[1] <= o[1][1]):
            return False
    return True


def getVertexPair(space, vertex):
    reals = ompl.util.vectorDouble()
    if vertex != ob.PlannerData.NO_VERTEX:  # 頂点が存在しない状態じゃなかったら
        space.copyToReals(reals, vertex.getState())
        return reals[0], reals[1]
    return None, None


def plot_result(state_space, path_result, planning_data):
    # 得られた解のの経由点を整理
    states = path_result.getStates()
    path_point_x = []
    path_point_y = []
    for s in states:
        path_point_x.append(s[0])
        path_point_y.append(s[1])

    # すべてのエッジを整理
    edge_list_x = []
    edge_list_y = []

    edge_list = ompl.util.vectorUint()
    for i in range(planning_data.numVertices()):
        n_edge = planning_data.getEdges(i, edge_list)
        for j in range(n_edge):
            x_1, y_1 = getVertexPair(state_space, planning_data.getVertex(i))
            x_2, y_2 = getVertexPair(state_space, planning_data.getVertex(edge_list[j]))
            x = [x_1, x_2]
            y = [y_1, y_2]
            edge_list_x.append(x)
            edge_list_y.append(y)

    # Figureを作成
    fig = plt.figure()
    ax = fig.add_subplot(111, aspect='equal')
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)

    # 障害物を描画
    for o in obstacle:
        rec1 = patches.Rectangle(
            o[0],
            o[1][0] - o[0][0],
            o[1][1] - o[0][1],
            edgecolor='#000000',
            facecolor='#f08080',
            fill=True
        )
        ax.add_patch(rec1)

    # 探索中に作成したエッジを描画
    for i in range(len(edge_list_x)):
        ax.plot(edge_list_x[i], edge_list_y[i], color="#808080", linewidth=0.5)

    # 得られた解を描画
    ax.plot(path_point_x, path_point_y, color="#ff0000")

    # スタートとゴールを追加
    ax.scatter(start_s[0], start_s[1], color="#ff8c00")
    ax.scatter(goal_s[0], goal_s[1], color="#ff8c00")

    # 画像保存
    fig.savefig("reuslt.png")


def plan():
    # create an Vector state space
    space = ob.RealVectorStateSpace(SPACE_DIM)

    # # set lower and upper bounds
    bounds = ob.RealVectorBounds(SPACE_DIM)
    for i in range(SPACE_DIM):
        bounds.setLow(i, -1)
        bounds.setHigh(i, 1)
    space.setBounds(bounds)

    # create a simple setup object
    ss = og.SimpleSetup(space)

    # set planner
    planner = og.RRTstar(ss.getSpaceInformation())
    ss.setPlanner(planner)

    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

    start = ob.State(space)
    start_state = start.get()
    for i in range(SPACE_DIM):
        start_state[i] = start_s[i]
        # print(start_state[i])

    goal = ob.State(space)
    goal_state = goal.get()
    for i in range(SPACE_DIM):
        goal_state[i] = goal_s[i]
        # print(goal_state[i])

    ss.setStartAndGoalStates(start, goal)

    # default parameters
    solved = ss.solve(1.0)

    if solved:
        # パスの簡単化を実施
        ss.simplifySolution()
        # 結果を取得
        path_result = ss.getSolutionPath()
        print(path_result)

        si = ss.getSpaceInformation()
        pdata = ob.PlannerData(si)
        ss.getPlannerData(pdata)

        space = path_result.getSpaceInformation().getStateSpace()
        plot_result(space, path_result, pdata)


if __name__ == "__main__":
    plan()
