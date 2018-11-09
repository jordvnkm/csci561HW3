# Jordan Minatogawa
# Algorithm will set expected cost to goal for each grid location. Reset these costs per car.
# Use expected costs to determine which way to go for each grid location.
# Run simulation for each car.

import numpy as np

# classes

# Wrapper that just contains start and end location.
class Car:
    def set_start_location(self, location_tuple):
        self.start_location = location_tuple

    def set_end_location(self, location_tuple):
        self.end_location = location_tuple


# methods

# main method. runs the whole program
def average_money_earned():
    input_file = open("input.txt")
    content = input_file.readlines()
    content = [line.strip() for line in content]
    # Parse content
    grid_size = int(content[0])
    num_cars = int(content[1])
    num_obstacles = int(content[2])
    
    # obstacles will be a set of tuples
    obstacles = set()
    for offset in range(0, num_obstacles):
        obstacle_string = content[3 + offset]
        obstacle_list = obstacle_string.split(",")
        obstacles.add((obstacle_list[0], obstacle_list[1]))

    current_index = 3 + num_obstacles
    # have cars in dictionary from num -> car
    cars_dict = {}
    for offset in range(0, num_cars):
        car = Car()
        start_string = content[current_index + offset]
        start_list = start_string.split(",")
        car.set_start_location((int(start_list[0]), int(start_list[1])))
        cars_dict[offset] = car

    current_index += num_cars
    for offset in range(0, num_cars):
        car = cars_dict[offset]
        end_string = content[current_index + offset]
        end_list = end_string.split(",")
        car.set_end_location((int(end_list[0]), int(end_list[1])))
        
    get_average_money_per_car(cars_dict, num_cars, grid_size, obstacles) 


# write to output the average money per car.
def get_average_money_per_car(cars_dict, num_cars, grid_size, obstacles):
    for index in range(0, num_cars):
        car = cars_dict[index]
        average_money = get_average_money(car, grid_size, obstacles)
        output_file = open("output.txt", "a")
        output_file.write(average_money + "\n")
        output_file.close()

# returns the average money per car.
def get_average_money(car, grid_size, obstacles):
    total = 0.0
    expected_cost_grid  = get_expected_cost_grid(car, grid_size, obstacles) 
    policy_grid = create_policy_grid(expected_cost_grid, grid_size)
    for seed in range(1, 11):
        total += get_money_earned(car, grid_size, obstacles, policy_grid, seed)
        
    return total/10.0

# runs one leg of the simulation based on seed.
def get_money_earned(car, grid_size, obstacles, policy_grid, seed):
    current_location = car.start_location
    # if we start in the end location, then return 100 (probably unneeded)
    if current_location == car.end_location:
        return 100

    money = 0.0
    np.random.seed(seed)   
    swerve = np.random.random_sample(1000000)
    k = 0
    while current_location != car.end_location:
        # use index 0 for X, and index 1 for Y.
        desired_move = policy_grid[current_location[0]][current_location[1]]
        actual_move = get_randomized_move(desired_move, swerve, k)
        current_location = get_next_location(grid_size, current_location, actual_move)
        # remove 1$ for gas
        money -= 1.0

        # remove 100$ for running into obstacle
        if current_location in obstacles:
            money -= 100.0
        k += 1

    # we have broken out of loop and thus have reached end_location
    money += 100.0
    return money

def turn_left(move):
    if move == "N":
        return "W"
    elif move == "S":
        return "E"
    elif move == "E":
        return "N"
    elif move == "W":
        return "S"

def turn_right(move):
    if move == "N":
        return "E"
    elif move == "S":
        return "W"
    elif move == "E":
        return "S"
    elif move == "W":
        return "N"

# returns a direction (N, S, E , W)
def get_randomized_move(desired_move, swerve, k):
    if swerve[k] > 0.7:
        if swerve[k] > 0.8:
            if swerve[k] > 0.9:
                return turn_left(turn_left(desired_move))
            else:
                return turn_left(desired_move)
        else:
            return turn_right(desired_move)

# returns the next grid coordinate based on current location and move.
def get_next_location(grid_size, current_location, actual_move):
    move_delta = get_move_delta_from_move(actual_move)
    next_location = (current_location[0] + move_delta[0], current_location[1] + move_delta[1])
    if is_valid_location(next_location, grid_size):
        return next_location
    else:
        return current_location


# Returns a bool indicating whether the location is in the grid.
def is_valid_location(location, grid_size):
    if location[0] < 0 or location[0] >= grid_size:
        output_file = open("output.txt", "a")
        output_file.write("location = " + str(location) + " answer: " + str(False) + " gridsize:" + str(grid_size) + "\n")
        output_file.close()
        return False
    if location[1] < 0 or location[1] >= grid_size:
        output_file = open("output.txt", "a")
        output_file.write("location = " + str(location) + " answer: " + str(False) + " gridsize:" + str(grid_size)+ "\n")
        output_file.close()
        return False
    output_file = open("output.txt", "a")
    output_file.write("location = " + str(location) + " answer: " + str(True)+ " gridsize:" + str(grid_size) + "\n")
    output_file.close()
    return True
    

# returns a list (x, y) delta
def get_move_delta_from_move(actual_move):
    if actual_move == "N":
        return (0, 1)
    elif actual_move == "S":
        return (0, -1)
    elif actual_move == "W":
        return (-1, 0)
    else:
        return (1, 0)

# creates a grid of strings (N, S, E, W) representing 
# which move to take in each location
def create_policy_grid(expected_cost_grid, grid_size):
    policy_grid = []
    for i in range(0, grid_size):
        column = [None] * grid_size
        policy_grid.append(column)

    for col in range(0, grid_size):
        for row in range(0, grid_size):
            policy_grid[col][row] = get_best_move(col, row, expected_cost_grid, grid_size)
            

# gets best move given the expected cost grid. (N, S , E , W)
def get_best_move(col, row, expected_cost_grid, grid_size):
    deltas = {"N": [0, 1], "S": [0, -1], "W":[-1, 0], "E":[1,0]}
    best_move = ""
    min_expected_cost = None
    for direction in deltas:
        delta = deltas[direction]
        new_col = col + delta[0]
        new_row = col + delta[1]
        if is_valid_location((new_col, new_row), grid_size):
            expected_cost = expected_cost_grid[new_col][new_row]
            if min_expected_cost == None or expected_cost < min_expected_cost:
                best_move = direction
                min_expected_cost = expected_cost
            elif expected_cost == min_expected_cost and is_higher_priority(direction, best_move):
                best_move = direction
    return best_move

    
# returns true if direction1 is higher priority than direction2
def is_higher_priority(direction1, direction2):
    priorities = ["N", "S", "E", "W"]
    return priorities.index(direction1) < priorities.index(direction2)


# creates an expected cost grid
def get_expected_cost_grid(car, grid_size, obstacles):
    # lowest cost grid will contain the smallest cost to get from
    # (x, y) to the cars end location.
    lowest_cost_grid = []
    for col in range(0, grid_size):
        for row in range(0, grid_size):
            column = [0] * grid_size;
            lowest_cost_grid.append(column)

    for col in range(0, grid_size):
        for row in range(0, grid_size):
            fill_lowest_cost_grid(lowest_cost_grid, col, row, car, obstacles)

    return expected_cost_grid_from_lowest_cost_grid(lowest_cost_grid, car)


# fills the lowest cost grid at col, row.
def fill_lowest_cost_grid(lowest_cost_grid, col, row, car, obstacles):
    visited = set()
    grid_size = len(lowest_cost_grid[0])
    output_file = open("output.txt", "a")
    output_file.write("grid size ==" + str(grid_size) + "\n")
    output_file.close()
    lowest_cost_grid[col][row] = get_cost_from_location(col, row, visited, car, obstacles, grid_size)

# returns the lowest cost from current col, row to cars end location.
def get_cost_from_location(col, row, visited, car, obstacles, grid_size):
    location = (col, row)
    if location == car.end_location:
        return -100

    cost = 1
    if location in obstacles:
        cost += 100

    visited.add(location)
    deltas = [[1,0], [0,1], [-1,0], [0,-1]]
    min_cost = 0
    for delta in deltas:
        new_col = col + delta[0]
        new_row = col + delta[1]
        new_location = (new_col, new_row)
        if is_valid_location(new_location, grid_size) and not (new_location in visited):
            min_cost = min(min_cost, get_cost_from_location(new_col, new_row, visited, car, obstacles, grid_size))
    visited.remove(location)
    return min_cost + cost

def expected_cost_grid_from_lowest_cost_grid(lowest_cost_grid, car):
    grid_size = len(lowest_cost_grid[0])
    expected_cost_grid = []
    for i in range(0, grid_size):
        column = [None] * grid_size
        expected_cost_grid.append(column)
    for col in range(0, grid_size):
        for row in range(0, grid_size):
            north_cost = get_north_cost(col, row, lowest_cost_grid, grid_size)
            south_cost = get_south_cost(col, row, lowest_cost_grid, grid_size)
            west_cost = get_west_cost(col, row, lowest_cost_grid, grid_size)
            east_cost = get_east_cost(col, row, lowest_cost_grid, grid_size)
            costs = [north_cost, south_cost, west_cost, east_cost]
            total = sum(costs)
            min_cost = min(costs)
            expected_cost = 0.7 * min_cost + 0.1*(total - min_cost)
            expected_cost_grid[col][row] = expected_cost
    return expected_cost_grid

def get_north_cost(col, row, lowest_cost_grid, grid_size):
    if is_valid_location((col, row + 1), grid_size):
        return lowest_cost_grid[col][row + 1]
    else:
        return lowest_cost_grid[col][row] + 1

def get_south_cost(col, row, lowest_cost_grid, grid_size):
    if is_valid_location((col, row - 1), grid_size):
        return lowest_cost_grid[col][row - 1]
    else:
        return lowest_cost_grid[col][row] + 1

def get_east_cost(col, row, lowest_cost_grid, grid_size):
    if is_valid_location((col + 1, row), grid_size):
        return lowest_cost_grid[col + 1][row]
    else:
        return lowest_cost_grid[col][row] + 1

def get_west_cost(col, row, lowest_cost_grid, grid_size):
    if is_valid_location((col - 1, row), grid_size):
        return lowest_cost_grid[col - 1][row]
    else:
        return lowest_cost_grid[col][row] + 1

if __name__ == "__main__":
    average_money_earned()


