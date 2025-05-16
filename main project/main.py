import math
import time
from collections import deque

import pygame
from game import AngryGame, PygameInit


def heuristic(grid, num_actions):
    hen_pos = env.get_hen_position(grid)
    eggs = env.get_egg_coordinate(grid)
    if AngryGame.is_win(grid):
        if eggs:
            slingshot_distance = 16
        else:
            return 100

    else:
        slingshot_pos = env.get_slingshot_position(grid)
        slingshot_distance = find_nearest_distance(grid, ['B', 'R'], slingshot_pos, 'H')

    queen_pos = env.get_queen_position(grid)
    pigs = env.get_pig_coordinate(grid)

    slingshot_distance = find_nearest_distance(grid, ['P', 'R'], hen_pos, 'S')

    queen_distance = find_nearest_distance(grid, ['B', 'R'], queen_pos, 'H')

    queen_factor = 0 if queen_distance > 4 else 0.1
    pigs_factor = len(pigs) * .5
    if eggs:
        dist_to_closest_egg = find_nearest_distance(grid, ['B', 'R', 'Q', 'S'], hen_pos, 'E')
        egg_factor = (10 / dist_to_closest_egg) - (len(eggs) * 10)
        slingshot_factor = 8 - len(eggs)
    else:
        egg_factor = 0
        slingshot_factor = 10
    # queen = -1000 pig = -200 egg=200 goal=400 action =-1
    heuristic_value = (egg_factor + queen_distance * queen_factor + pigs_factor -
                       num_actions + slingshot_factor / slingshot_distance)
    return heuristic_value


def find_nearest_distance(grid, obstacles, start, goal):
    rows, cols = len(grid), len(grid[0])

    def is_valid(x, y):
        return (0 <= x < rows) and (0 <= y < cols) and (grid[x][y] not in obstacles)


    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up
    queue = deque([(start[0], start[1], 0)])  # (row, col, distance)
    visited = set()
    visited.add(start)

    while queue:
        x, y, dist = queue.popleft()

        if grid[x][y] == goal:
            return dist

        for dx, dy in directions:
            nx, ny = x + dx, y + dy

            if is_valid(nx, ny) and (nx, ny) not in visited:
                visited.add((nx, ny))
                queue.append((nx, ny, dist + 1))

    return 18  # If end is unreachable


def min_max(grid, depth, is_max, alpha, beta, num_actions):
    if depth == 0 or AngryGame.is_win(grid) or AngryGame.is_lose(grid, num_actions):
        return heuristic(grid, num_actions), None
        # return env.calculate_score(grid, num_actions), None

    if is_max:
        max_eval = -math.inf
        best_action = None
        successors = env.generate_hen_successors(grid)
        for successor_grid, action in successors:
            eval_score, _ = min_max(successor_grid, depth - 1, False, alpha, beta, num_actions + 1)
            if eval_score > max_eval:
                max_eval = eval_score
                best_action = action

            alpha = max(alpha, eval_score)
            if beta <= alpha:
                break
        return max_eval, best_action
    else:
        min_eval = math.inf
        successors = env.generate_queen_successors(grid)
        for successor_grid, _ in successors:
            eval_score, _ = min_max(successor_grid, depth - 1, True, alpha, beta, num_actions)
            min_eval = min(min_eval, eval_score)
            beta = min(beta, eval_score)
            if beta <= alpha:
                break
        return min_eval, None


if __name__ == "__main__":

    env = AngryGame(template='simple')

    screen, clock = PygameInit.initialization()
    FPS = 9

    env.reset()
    counter = 0

    running = True
    while running:
        if AngryGame.is_win(env.grid) or AngryGame.is_lose(env.grid, env.num_actions):
            running = False

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        if counter % 2 == 0:
            _, action = min_max(env.grid, depth=7, is_max=True, alpha=-math.inf, beta=math.inf,
                                num_actions=env.num_actions)
            if action is not None:
                env.hen_step(action)
            env.render(screen)
            if AngryGame.is_win(env.grid):
                running = False

        if counter % 2 == 1:
            env.queen_step()
            env.render(screen)
            if AngryGame.is_lose(env.grid, env.num_actions):
                running = False

        counter += 1
        pygame.display.flip()
        clock.tick(FPS)
        print(f'Current Score == {AngryGame.calculate_score(env.grid, env.num_actions)}')

    pygame.quit()
