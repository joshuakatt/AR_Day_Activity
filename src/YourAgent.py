from src.DriveInterface import DriveInterface
from src.DriveState import DriveState
from src.Constants import DriveMove, SensorData
import heapq  # for priority queue

class DfsSolverAgent(DriveInterface):



    def __init__(self, game_id: int, is_advanced_mode: bool):
        # Constructor for player
        # Player ID will always be 0
        self.game_id  = game_id
        self.path = []
        self.field_limits = []
        self.path_move_index = 0
        self.need_to_find_target_pod = is_advanced_mode
    
    def heuristic(self, state: DriveState, goal: list[int]) -> int:
        # Manhattan distance
        return abs(state.x - goal[0]) + abs(state.y - goal[1])

    def get_next_move(self, sensor_data: dict) -> DriveMove:
    # Main function called by game orchestrator
    # Returns a DriveMove enum value
        if len(self.path) == 0:
            if self.need_to_find_target_pod:
                # Advanced mode - Need to find the target pod and bring it to the goal
                self.dfs_solve_path_to_goal(sensor_data, sensor_data[SensorData.TARGET_POD_LOCATION])
                
                # Add the LIFT_POD move to the path
                self.path.append(DriveMove.LIFT_POD)
                
                # Store the current path to later merge with the path to the goal
                path_to_pod = self.path.copy()
                
                # Clear the current path and find the path to the goal
                self.path = []
                self.dfs_solve_path_to_goal(sensor_data, sensor_data[SensorData.GOAL_LOCATION])
                
                # Merge the path to the pod with the path to the goal
                self.path = path_to_pod + self.path
            else:
                self.dfs_solve_path_to_goal(sensor_data, sensor_data[SensorData.GOAL_LOCATION])

        next_move, next_state = self.get_move_for_next_state_in_path()
        if self.will_next_state_collide(next_state, sensor_data):
            self.path_move_index -= 1
            # print('Next move would have crashed player, waiting 1 move.')
            return DriveMove.NONE
        else:
            return next_move

    def will_next_state_collide(self, next_state, sensor_data: dict) -> bool:
        for drive_state in sensor_data[SensorData.DRIVE_LOCATIONS]:
            if drive_state[0] == next_state.x and drive_state[1] == next_state.y:
                return True
        return False


    def get_move_for_next_state_in_path(self) -> DriveMove:
        # Function to find the move which will get the player to the next state in self.path
        current_state = self.path[self.path_move_index]
        self.path_move_index += 1
        next_state = self.path[self.path_move_index]

        for move in DriveMove:
            if current_state.get_next_state_from_move(move) == next_state.to_tuple():
                return move, next_state

        print('WARN next move could not be found')
        return DriveMove.NONE, next_state

    def dfs_solve_path_to_goal(self, sensor_data: dict, goal: list[int]):
        start_state = DriveState(x=sensor_data[SensorData.PLAYER_LOCATION][0], y=sensor_data[SensorData.PLAYER_LOCATION][1])

        open_list = [(self.heuristic(start_state, goal), [start_state])]
        g_values = {start_state.to_tuple(): 0}
        visited_states = set()

        while open_list:
            _, current_path = heapq.heappop(open_list)
            curr_state = current_path[-1]

            if curr_state.x == goal[0] and curr_state.y == goal[1]:
                self.path = current_path
                return

            visited_states.add(curr_state.to_tuple())

            for next_state in self.list_all_next_possible_states(curr_state, sensor_data):
                new_g = g_values[curr_state.to_tuple()] + 1  # assuming cost for each move is 1
                if next_state.to_tuple() not in g_values or new_g < g_values[next_state.to_tuple()]:
                    g_values[next_state.to_tuple()] = new_g
                    f = new_g + self.heuristic(next_state, goal)
                    if next_state.to_tuple() not in visited_states:
                        heapq.heappush(open_list, (f, current_path + [next_state]))

        print('WARN Could not find solution from A* solver')


    def list_all_next_possible_states(self, state: DriveState, sensor_data: dict) -> list[DriveState]:
        next_states = []
        for move in DriveMove:
            x, y = state.get_next_state_from_move(move)
            next_state = DriveState(x=x, y=y)
            if not self.will_next_state_collide(next_state, sensor_data) and self.is_state_in_bounds(next_state, sensor_data):
                next_states.append(next_state)

        return next_states

    def is_state_in_bounds(self, state: DriveState, sensor_data: dict) -> bool:
        # Checks if state argument is not a field wall
        return [state.x, state.y] not in sensor_data[SensorData.FIELD_BOUNDARIES]

    def is_player_drive_carrying_a_pod(self, sensor_data: dict) -> bool:
        # Checks if player game id is the first value in any of the entries in SensorData.DRIVE_LIFTED_POD_PAIRS
        return self.game_id in [drive_pod_pair[0] for drive_pod_pair in sensor_data[SensorData.DRIVE_LIFTED_POD_PAIRS]]
            