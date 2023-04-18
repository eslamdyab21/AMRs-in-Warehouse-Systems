import numpy as np

def convert_warehouse_map_to_astart_map(warehouse_map, robot_id = None):
    # print(warehouse_map)

    for x in range(0, warehouse_map.shape[0]):
        for y in range(0, warehouse_map.shape[1]):
            if warehouse_map[x,y] == robot_id:
                warehouse_map[x,y] = 0

            elif warehouse_map[x,y] != 0:

                if warehouse_map[x,y][0] == 'S':
                    warehouse_map[x,y] = 0

                else:
                    warehouse_map[x,y] = 1

    astar_map = warehouse_map.astype('int')

    # print('-----------------------------------')
    # print(astar_map)
    return astar_map