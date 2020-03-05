import numpy as np

NUM_LAYERS = 12
LAYER_WIDTH = 39
LAYER_HEIGHT = 39

def assign(obs, head, x, y, l, v):
    s_x = x - head['x']
    s_y = y - head['y']

    s_x += LAYER_WIDTH // 2
    s_y += LAYER_HEIGHT // 2

    if s_x > 0 and s_x < LAYER_WIDTH and s_y > 0 and s_y < LAYER_HEIGHT:
        obs[l*LAYER_WIDTH*LAYER_HEIGHT + s_x * LAYER_HEIGHT + s_y] += v


def make_input(data):
    """ Method to transform the starter snake input into the correct format for our trained model """
    obs = np.zeros((LAYER_WIDTH * LAYER_HEIGHT * NUM_LAYERS), dtype=np.uint8)
    
    player_snake = data['you']
    head = player_snake['body'][0]

    # Assign the head for the player
    assign(obs, head, player_snake['body'][0]['x'], player_snake['body'][0]['y'], 8, 1)

    for snake in data['board']['snakes']:
        assign(obs, head, snake['body'][0]['x'], snake['body'][0]['y'], 0, snake['health'])

        i = 1
        for segment in reversed(snake['body']):
            assign(obs, head, segment['x'], segment['y'], 1, 1)
            assign(obs, head, segment['x'], segment['y'], 2, min(i, 255))
            i += 1

            if snake['id'] != player_snake['id'] and NUM_LAYERS == 12:
                len_enemy = len(snake['body'])
                len_player = len(player_snake['body'])
                if len_enemy >= len_player:
                    assign(obs, head, segment['x'], segment['y'], 10, 1 + len_enemy - len_player)
                else:
                    assign(obs, head, segment['x'], segment['y'], 11, len_player - len_enemy)


        # Check for matching tails
        tail_1 = snake['body'][-1]
        tail_2 = snake['body'][-2]

        if tail_1['x'] == tail_2['x'] and tail_1['y'] == tail_2['y']:
            # Mark the doubled tail
            assign(obs, head, tail_1['x'], tail_1['y'], 9, 1)


        if snake['id'] != player_snake['id']:
            is_bigger = 0
            if len(snake['body']) >= len(player_snake['body']):
                is_bigger = 1
            snake_head = snake['body'][0]
            assign(obs, head, snake_head['x'], snake_head['y'], 3, is_bigger)

    for food in data['board']['food']:
        assign(obs, head, food['x'], food['y'], 4, 1)

    for x in range(data['board']['width']):
        for y in range(data['board']['height']):
            assign(obs, head, x, y, 5, 1)

    _fill_ownership(data, obs, head)    

    inp = obs.reshape(1, NUM_LAYERS, LAYER_WIDTH, LAYER_HEIGHT)

    return inp


def _fill_ownership(data, obs, head):
    """ To match the format from gym-battlesnake """
    print("Starting fill")
    grid = np.zeros((39,39,5), dtype=np.int32)
    # 0: distance
    # 1: owner
    # 2: body
    # 3: contested
    # 4: visited

    # Fill in body parts
    for snake in data['board']['snakes']:
        for segment in snake['body']:
            grid[segment['x'], segment['y'], 2] = 1

    id_map = {}
    for i, snake in enumerate(data['board']['snakes']):
        id_map[snake['id']] = i
    
    you_id = id_map[data['you']['id']]

    for snake in data['board']['snakes']:
        snake_id = id_map[snake['id']]
        seen = np.zeros((39,39), dtype=np.bool)

        snake_head = snake['body'][0]

        queue = []

        # Add snake head
        queue.append((snake_head['x'], snake_head['y'], 0))

        while len(queue) > 0:
            node = queue.pop(0)
            seen[node[0], node[1]] = True

            up = (node[0], node[1] - 1, node[2] + 1)
            right = (node[0] + 1, node[1], node[2] + 1)
            down = (node[0], node[1] + 1, node[2] + 1)
            left = (node[0] - 1, node[1], node[2] + 1)

            if up[1] >= 0 and seen[up[0], up[1]] == False:
                if grid[up[0], up[1], 2] == 0:
                    seen[up[0], up[1]] = 1
                    queue.append(up)
            if right[0] < data['board']['width'] and seen[right[0], right[1]] == False:
                if grid[right[0], right[1], 2] == 0:
                    seen[right[0], right[1]] = 1
                    queue.append(right)
            if down[1] < data['board']['height'] and seen[down[0], down[1]] == False:
                if grid[down[0], down[1], 2] == 0:
                    seen[down[0], down[1]] = 1
                    queue.append(down)
            if left[0] >= 0 and seen[left[0], left[1]] == False:
                if grid[left[0], left[1], 2] == 0:
                    seen[left[0], left[1]] = 1
                    queue.append(left)

            if grid[node[0], node[1], 4] == 0:
                # Unvisited
                grid[node[0], node[1], 1] = snake_id # Set the owner
                grid[node[0], node[1], 0] = node[2] # SET THE DISTANCE
                grid[node[0], node[1], 4] = 1 # SET VISITED
            elif node[2] < grid[node[0], node[1], 0]:
                # We're closer
                grid[node[0], node[1], 1] = snake_id # SET THE OWNER
                grid[node[0], node[1], 0] = node[2] # SET THE DISTANCE
                grid[node[0], node[1], 3] = 0 # SET Contested = false
            elif grid[node[0], node[1], 0] == node[2]:
                # Equal distance
                if grid[node[0], node[1], 1]  != snake_id:
                    grid[node[0], node[1], 3] = 1

    for x in range(data['board']['width']):
        for y in range(data['board']['height']):
            if grid[x, y, 3] == False and grid[x, y, 4] == 1:
                if grid[x, y, 1] == you_id:
                    assign(obs, head, x, y, 6, 1)
                else:
                    assign(obs, head, x, y, 7, 1)

    print("Done fill")