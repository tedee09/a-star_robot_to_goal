from controller import Supervisor
import random

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Ukuran arena
arena_size = 4  
min_gap = 0.9  
children_field = supervisor.getRoot().getField("children")

# Simpan daftar dinding
walls = []

def is_valid_position(x, y, length, is_vertical, min_distance=min_gap):
    for wx, wy, w_length, w_vertical in walls:
        if is_vertical == w_vertical:
            if is_vertical:
                if abs(x - wx) < min_distance and not (y + length / 2 < wy - w_length / 2 or y - length / 2 > wy + w_length / 2):
                    return False
            else:
                if abs(y - wy) < min_distance and not (x + length / 2 < wx - w_length / 2 or x - length / 2 > wx + w_length / 2):
                    return False
        else:
            if abs(x - wx) < min_distance and abs(y - wy) < min_distance:
                return False
    return True

def create_obstacle():
    max_attempts = 100  
    for _ in range(max_attempts):
        is_vertical = random.choice([True, False])
        length = random.uniform(0.5, 2.0)

        if is_vertical:
            x = random.uniform(-arena_size / 2 + 0.1, arena_size / 2 - 0.1)
            y = random.uniform(-arena_size / 2 + length / 2, arena_size / 2 - length / 2)
        else:
            x = random.uniform(-arena_size / 2 + length / 2, arena_size / 2 - length / 2)
            y = random.uniform(-arena_size / 2 + 0.1, arena_size / 2 - 0.1)

        if is_valid_position(x, y, length, is_vertical):
            walls.append((x, y, length, is_vertical))
            width = 0.03
            height = 0.1
            size_x, size_y = (width, length) if is_vertical else (length, width)

            obstacle_def = f"""
            Solid {{
              translation {x} {y} 0.05
              children [
                Shape {{
                  appearance Appearance {{
                    material Material {{ diffuseColor 0.5 0.5 0.5 }}
                  }}
                  geometry Box {{ size {size_x} {size_y} {height} }}
                }}
              ]
              name "wall"
              boundingObject Box {{ size {size_x} {size_y} {height} }}
            }}
            """
            children_field.importMFNodeFromString(-1, obstacle_def)
            return  

# Hapus semua dinding lama sebelum membuat maze baru
for i in range(children_field.getCount() - 1, -1, -1):
    node = children_field.getMFNode(i)
    if node.getTypeName() == "Solid":
        children_field.removeMF(i)

# Buat maze dengan dinding yang tidak bertumpukan dan memiliki jarak minimum
num_walls = 20
for _ in range(num_walls):
    create_obstacle()
