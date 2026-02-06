import numpy as np
import random

import carb

from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from omni.isaac.core.world import World
from isaacsim.core.api.objects import (
    DynamicCuboid, VisualCuboid,
    DynamicSphere, VisualSphere,
    DynamicCylinder, VisualCylinder,
    DynamicCone, VisualCone,
    DynamicCapsule, VisualCapsule
)
from pxr import UsdLux


def customize_world(world: World, pg: PegasusInterface):
    """
    Customize the Isaac Sim world with random primitive obstacles.
    
    Creates a scene with various primitive shapes (cubes, spheres, cylinders, 
    cones, capsules) scattered throughout the environment with random scales,
    orientations, and colors.
    """
    carb.log_info("Adding default ground plane to the scene.")
    world.scene.add_default_ground_plane()
    
    # Add distant light for better scene illumination
    carb.log_info("Adding distant light to the scene.")
    distant_light = UsdLux.DistantLight.Define(world.scene.stage, "/World/DistantLight")
    distant_light.CreateIntensityAttr(1000)
    
    # Configuration for random obstacles
    num_obstacles = 50  # Total number of obstacles to create
    scene_bounds = {
        'x_min': -10, 'x_max': 10,
        'y_min': -10, 'y_max': 10,
        'z_min': 0.1, 'z_max': 8
    }
    
    # Shape types with their corresponding classes (mix of dynamic and visual)
    shape_types = [
        ('cube', DynamicCuboid, VisualCuboid),
        ('sphere', DynamicSphere, VisualSphere),
        ('cylinder', DynamicCylinder, VisualCylinder),
        ('cone', DynamicCone, VisualCone),
        ('capsule', DynamicCapsule, VisualCapsule)
    ]
    
    carb.log_info(f"Creating {num_obstacles} random primitive obstacles...")
    
    for i in range(num_obstacles):
        # Randomly choose shape type
        shape_name, dynamic_class, visual_class = random.choice(shape_types)
        
        # Randomly decide if this should be dynamic (physics-enabled) or visual only
        use_dynamic = random.choice([True, False])
        shape_class = dynamic_class if use_dynamic else visual_class
        
        # Generate random position within scene bounds
        position = np.array([
            random.uniform(scene_bounds['x_min'], scene_bounds['x_max']),
            random.uniform(scene_bounds['y_min'], scene_bounds['y_max']),
            random.uniform(scene_bounds['z_min'], scene_bounds['z_max'])
        ])
        
        # Generate random scale (0.2 to 2.0 for each axis)
        scale = np.array([
            random.uniform(0.2, 2.0),
            random.uniform(0.2, 2.0),
            random.uniform(0.2, 2.0)
        ])
        
        # Generate random orientation (quaternion)
        # Create random rotation around each axis
        roll = random.uniform(0, 2 * np.pi)
        pitch = random.uniform(0, 2 * np.pi)
        yaw = random.uniform(0, 2 * np.pi)
        
        # Convert Euler angles to quaternion (w, x, y, z)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        
        orientation = np.array([
            cr * cp * cy + sr * sp * sy,  # w
            sr * cp * cy - cr * sp * sy,  # x
            cr * sp * cy + sr * cp * sy,  # y
            cr * cp * sy - sr * sp * cy   # z
        ])
        
        # Generate random color (RGB values 0-255)
        color = np.array([
            random.randint(0, 255),
            random.randint(0, 255),
            random.randint(0, 255)
        ])
        
        # Create unique prim path and name
        prim_path = f"/World/random_obstacle_{shape_name}_{i}"
        name = f"obstacle_{shape_name}_{i}"
        
        # Shape-specific parameters
        common_params = {
            'prim_path': prim_path,
            'name': name,
            'position': position,
            'scale': scale,
            'orientation': orientation,
            'color': color
        }
        
        # Add dynamic-specific parameters
        if use_dynamic:
            common_params['mass'] = random.uniform(0.1, 5.0)  # Random mass between 0.1 and 5 kg
            # Add some random initial velocity for dynamic objects (occasionally)
            if random.random() < 0.3:  # 30% chance of initial velocity
                common_params['linear_velocity'] = np.array([
                    random.uniform(-2, 2),
                    random.uniform(-2, 2),
                    random.uniform(-1, 1)
                ])
        
        # Create shape-specific parameters
        if shape_name == 'cube':
            # For cubes, use a base size that gets scaled
            common_params['size'] = random.uniform(0.5, 1.5)
            
        elif shape_name == 'sphere':
            # For spheres, use radius
            common_params['radius'] = random.uniform(0.3, 1.0)
            # Remove scale for spheres as radius controls size
            del common_params['scale']
            
        elif shape_name == 'cylinder':
            # For cylinders, use radius and height
            common_params['radius'] = random.uniform(0.2, 0.8)
            common_params['height'] = random.uniform(0.5, 2.0)
            # Remove scale for cylinders as radius/height control size
            del common_params['scale']
            
        elif shape_name == 'cone':
            # For cones, use radius and height
            common_params['radius'] = random.uniform(0.2, 0.8)
            common_params['height'] = random.uniform(0.5, 2.0)
            # Remove scale for cones as radius/height control size
            del common_params['scale']
            
        elif shape_name == 'capsule':
            # For capsules, use radius and height
            common_params['radius'] = random.uniform(0.2, 0.6)
            common_params['height'] = random.uniform(0.8, 2.0)
            # Remove scale for capsules as radius/height control size
            del common_params['scale']
        
        try:
            # Create and add the shape to the scene
            obstacle = shape_class(**common_params)
            world.scene.add(obstacle)
            
            carb.log_info(f"Created {shape_name} obstacle {i+1}/{num_obstacles} at position {position}")
            
        except Exception as e:
            carb.log_warn(f"Failed to create {shape_name} obstacle {i}: {str(e)}")
            continue
    
    carb.log_info("Finished creating random primitive obstacles.")