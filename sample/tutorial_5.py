import ompl
from ompl import base as ob
from ompl import geometric as og

from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation
import numpy as np

SPACE_DIM = 6

min_bound = -1.5
max_bound = 1.5

robot_x = 0.3
robot_y = 0.4
robot_z = 0.2

robot_r = np.sqrt(robot_x**2 + robot_y**2 + robot_z**2)

obstacle = [(0, 0, 0, 0.75), (0.5, 0.5, 0.5, 0.3), (-0.5, -0.5, 0.5, 0.2)]
start_s = (-0.90, 0.90, 0.90, 0, 0, 0)
goal_s = (0.90, -0.90, -0.90, np.pi / 2, np.pi / 4, 0)


def isStateValid(state):
    # 指定された矩形に含まれているかを確認。
    for o in obstacle:
        distance = np.sqrt((state[0] - o[0])**2 + (state[1] - o[1])**2 + (state[2] - o[2])**2)
        if distance < robot_r + o[3]:
            return False
    return True


def getVertexPair(space, vertex):
    reals = ompl.util.vectorDouble()
    if vertex != ob.PlannerData.NO_VERTEX:  # 頂点が存在しない状態じゃなかったら
        space.copyToReals(reals, vertex.getState())
        return reals[0], reals[1], reals[2]
    return None, None, None


def draw_box(position, rotation):
    euler = np.array(rotation)
    rot = Rotation.from_euler('XYX', euler)

    edge_list = [[robot_x, robot_y, robot_z],
                 [robot_x, -robot_y, robot_z],
                 [robot_x, robot_y, -robot_z],
                 [robot_x, -robot_y, -robot_z],
                 [-robot_x, robot_y, robot_z],
                 [-robot_x, -robot_y, robot_z],
                 [-robot_x, robot_y, -robot_z],
                 [-robot_x, -robot_y, -robot_z]]
    rotation = []
    for e in edge_list:
        rotation.append(rot.apply(np.array(e)) + np.array(position))
    rotation = np.array(rotation)
    # print(rotation)

    result = [[rotation[0], rotation[1], rotation[3], rotation[2]],
              [rotation[0], rotation[2], rotation[6], rotation[4]],
              [rotation[0], rotation[1], rotation[5], rotation[4]],
              [rotation[1], rotation[3], rotation[7], rotation[5]],
              [rotation[2], rotation[3], rotation[7], rotation[6]],
              [rotation[4], rotation[5], rotation[7], rotation[6]]]
    return result


def draw_sphere(point, r):
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)

    x = r * np.outer(np.cos(u), np.sin(v)) + point[0]
    y = r * np.outer(np.sin(u), np.sin(v)) + point[1]
    z = r * np.outer(np.ones(np.size(u)), np.cos(v)) + point[2]
    return [x, y, z]


def plot_result(state_space, path_result, planning_data):
    # 得られた解のの経由点を整理
    states = path_result.getStates()
    waypoints = []

    for s in states:
        state = []
        for i in range(SPACE_DIM):
            state.append(s[i])
        waypoints.append(state)
    waypoints = np.array(waypoints)

    # すべてのエッジを整理
    edge_list_x = []
    edge_list_y = []
    edge_list_z = []
    edge_list = ompl.util.vectorUint()
    for i in range(planning_data.numVertices()):
        n_edge = planning_data.getEdges(i, edge_list)
        for j in range(n_edge):
            x_1, y_1, z_1 = getVertexPair(state_space, planning_data.getVertex(i))
            x_2, y_2, z_2 = getVertexPair(state_space, planning_data.getVertex(edge_list[j]))
            x = [x_1, x_2]
            y = [y_1, y_2]
            z = [z_1, z_2]
            edge_list_x.append(x)
            edge_list_y.append(y)
            edge_list_z.append(z)

    # Figureを作成
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(min_bound * 1.5, max_bound * 1.5)
    ax.set_ylim(min_bound * 1.5, max_bound * 1.5)
    ax.set_zlim(min_bound * 1.5, max_bound * 1.5)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # 障害物を描画
    for o in obstacle:
        point_sphere = draw_sphere(o[0:3], o[3])
        ax.plot_surface(point_sphere[0], point_sphere[1], point_sphere[2], rstride=4, cstride=4, color='#ff69b4', linewidth=0, alpha=0.5)

    # 探索中に作成したエッジを描画
    for i in range(len(edge_list_x)):
        ax.plot(edge_list_x[i], edge_list_y[i], edge_list_z[i], color="#808080", linewidth=0.5)

    # 得られた解を描画
    tr_waypoints = waypoints.T
    ax.plot(tr_waypoints[0], tr_waypoints[1], tr_waypoints[2], color="#ff0000")
    for point in waypoints:
        pos = point[0:3]
        rot = point[3:6]
        result = draw_box(pos, rot)
        ax.add_collection3d(
            Poly3DCollection(
                result,
                facecolors='#4169e1',
                linewidths=1,
                edgecolors='#000000',
                alpha=.25))
    # スタートとゴールを追加
    ax.scatter3D(start_s[0], start_s[1], start_s[2], color="#ff8c00")
    ax.scatter3D(goal_s[0], goal_s[1], goal_s[2], color="#ff8c00")

    ax.set_box_aspect((1, 1, 1))
    # 画像保存
    plt.show()
    fig.savefig("reuslt.png")


def plan():
    # create an Vector state space
    space = ob.RealVectorStateSpace(SPACE_DIM)

    # # set lower and upper bounds
    bounds = ob.RealVectorBounds(SPACE_DIM)
    for i in range(SPACE_DIM - 3):
        bounds.setLow(i, min_bound)
        bounds.setHigh(i, max_bound)
    for i in range(SPACE_DIM - 3):
        bounds.setLow(i + 3, -np.pi)
        bounds.setHigh(i + 3, np.pi)
    space.setBounds(bounds)
    space.setBounds(bounds)

    # create a simple setup object
    ss = og.SimpleSetup(space)

    # set planner
    planner = og.RRTstar(ss.getSpaceInformation())
    ss.setPlanner(planner)

    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

    start = ob.State(space)
    # start.random()
    start_state = start.get()
    for i in range(SPACE_DIM):
        start_state[i] = start_s[i]
    # print(start_state[i])

    goal = ob.State(space)
    # goal.random()
    goal_state = goal.get()
    for i in range(SPACE_DIM):
        goal_state[i] = goal_s[i]
    # print(goal_state[i])

    ss.setStartAndGoalStates(start, goal)

    # default parameters
    solved = ss.solve(4.0)

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
