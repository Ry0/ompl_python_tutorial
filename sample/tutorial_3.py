from ompl import base as ob
from ompl import geometric as og

SPACE_DIM = 3


def isStateValid(state):
    print("valid check start.")
    for i in range(SPACE_DIM):
        print(state[i])
    print("valid check end.")
    return True


def plan():
    # create an Vector state space
    space = ob.RealVectorStateSpace(SPACE_DIM)
    # print(space)

    # # set lower and upper bounds
    bounds = ob.RealVectorBounds(SPACE_DIM)
    for i in range(SPACE_DIM):
        bounds.setLow(i, -1)
        bounds.setHigh(i, 1)
    space.setBounds(bounds)

    # create a simple setup object
    ss = og.SimpleSetup(space)

    # set planner
    planner = og.RRTConnect(ss.getSpaceInformation())
    ss.setPlanner(planner)

    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

    start = ob.State(space)
    # we can pick a random start state...
    start.random()
    start_state = start.get()
    for i in range(SPACE_DIM):
        # start_state[i] = 0.1
        print(start_state[i])

    goal = ob.State(space)
    # we can pick a random goal state...
    goal.random()

    ss.setStartAndGoalStates(start, goal)

    # default parameters
    solved = ss.solve(1.0)

    if solved:
        # try to shorten the path
        ss.simplifySolution()
        # print the simplified path
        print(ss.getSolutionPath())


if __name__ == "__main__":
    plan()
