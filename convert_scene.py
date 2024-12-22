from plyfile import PlyData, PlyElement
import numpy as np
import open3d as o3d
import os

def clip_mesh(mesh, lower_bound=0.04, upper_bound=1.5):
    
    vertices = np.asarray(mesh.vertices)
    triangles = mesh.triangles
    colors = mesh.vertex_colors

    triangles_to_keep = []

    for i, triangle in enumerate(triangles):
        v1, v2, v3 = vertices[triangle]
        
        if v1[2] > lower_bound and v2[2] > lower_bound and v3[2] > lower_bound and v1[2] < upper_bound and v2[2] < upper_bound and v3[2] < upper_bound:
            triangles_to_keep.append(triangle)

    filtered_mesh = o3d.geometry.TriangleMesh()
    filtered_mesh.vertices = mesh.vertices
    triangles_to_keep = np.array(triangles_to_keep)
    filtered_mesh.triangles = o3d.utility.Vector3iVector(triangles_to_keep)
    filtered_mesh.vertex_colors = colors

    # o3d.visualization.draw_geometries([filtered_mesh])
    return filtered_mesh

def create_urdf(mesh, scene_root, scene_name):
    clip_mesh = scene_name + "_mesh_clipped.obj"
    urdf_content = f"""
<?xml version="1.0"?>
<robot name="object">
    <link name="object">
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <geometry>
                <mesh filename="{clip_mesh}" scale="1.0 1.0 1.0"/>
            </geometry>
            <material name="slider_mat">
                <color rgba="0.6551257899944148 0.27566008875968395 0.43912572554665763 1"/>
            </material>
        </visual>

        <collision>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <geometry>
                <mesh filename="{clip_mesh}" scale="1.0 1.0 1.0"/>
            </geometry>
        </collision>

        <inertial>
            <density value="0.0"/>
        </inertial>
    </link>
</robot>"""
    urdf_path = os.path.join(scene_root, scene_name + ".urdf")
    output_path = os.path.join(scene_root, clip_mesh)
    with open(urdf_path, 'w') as file:
        file.write(urdf_content)
    o3d.io.write_triangle_mesh(output_path, mesh)

def create_npy(scene_root, scene_name):
    data = {
        'translation': np.array([[0.0, 0.25, 0.0]], dtype=np.float32),
        'translation_offset': np.array([[0.0, 0.0, 0.0]], dtype=np.float32),
        'rotation': np.array([[0.0, 0.0, 0.0, 1.0]], dtype=np.float32),
        'rotation_offset': np.array([[0.0, 0.0, 0.7071068, -0.7071068]], dtype=np.float32)
    }
    npy_path = os.path.join(scene_root, scene_name + ".npy")
    np.save(npy_path, data)

root_path = "/home/bowie/Desktop/scanAPP/data/scene"
scene_name = "scene"
scene_root = os.path.join(root_path, scene_name)
mesh_name = scene_name + "_mesh.ply"
input_mesh = os.path.join(scene_root, mesh_name)

mesh = o3d.io.read_triangle_mesh(input_mesh)

mesh = clip_mesh(mesh)

create_urdf(mesh, scene_root, scene_name)

create_npy(scene_root, scene_name)