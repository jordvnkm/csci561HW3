# Jordan Minatogawa
# Algorithm will set expected cost to goal for each grid location. Reset these costs per car.
# Use expected costs to determine which way to go for each grid location.
# Run simulation for each car.

#classes

class Car:
    def set_start_location(self, location_tuple):
        self.start_location = location_tuple

    def set_end_location(self, location_tuple):
        self.end_location = location_tuple

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
        car.set_start_location((start_list[0], start_list[1]))
        cars_dict[offset] = car

    current_index += num_cars
    for offset in range(0, num_cars):
        car = cars_dict[offset]
        end_string = content[current_index + offset]
        end_list = end_string.split(",")
        car.set_end_location((end_list[0], end_list[1]))
        
    get_average_money_per_car(cars_dict, num_cars, grid_size, obstacles) 


def get_average_money_per_car(cars_dict, num_cars, grid_size, obstacles):
    for index in range(0, num_cars):
        car = cars_dict[index]
        average_money = get_average_money(car, grid_size, obstacles)
        output_file = open("output.txt", "a")
        output_file.write(average_money + "\n")
        output_file.close()

def get_average_money(car, grid_size, obstacles):
    total = 0.0
    for seed in range(1, 11):
        total += get_money_earned(car, grid_size, obstacles, seed)
        
    return total/10.0

def get_money_earned(car, grid_size, obstacles, seed):
    expected_cost_grid  = get_expected_cost_grid(car, grid_size, obstacles) 


def get_expected_cost_grid(car, grid_size, obstacles):




