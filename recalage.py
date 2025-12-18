import open3d as o3d
import numpy as np
import copy
import time

mesh_orig = o3d.io.read_triangle_mesh("cat1_un4.obj")
mesh_orig.compute_vertex_normals()
P_orig = np.asarray(mesh_orig.vertices)

def rot_x(theta):
    return np.array([[1,0,0],
                     [0,np.cos(theta),-np.sin(theta)],
                     [0,np.sin(theta),np.cos(theta)]])
def rot_y(theta):
    return np.array([[np.cos(theta),0,np.sin(theta)],
                     [0,1,0],
                     [-np.sin(theta),0,np.cos(theta)]])
def rot_z(theta):
    return np.array([[np.cos(theta),-np.sin(theta),0],
                     [np.sin(theta), np.cos(theta),0],
                     [0,0,1]])

theta_x, theta_y, theta_z = np.radians([30,90,45])
T_initial = np.array([50,20,-10])
R_manual = rot_z(theta_z) @ rot_y(theta_y) @ rot_x(theta_x)
P_transformed = (R_manual @ P_orig.T).T + T_initial

pcd_src = o3d.geometry.PointCloud()
pcd_src.points = o3d.utility.Vector3dVector(P_transformed)
pcd_src.paint_uniform_color([1,0,0])  

pcd_tgt = o3d.geometry.PointCloud()
pcd_tgt.points = o3d.utility.Vector3dVector(P_orig)
pcd_tgt.paint_uniform_color([0,0,1])  


threshold = 50.0
reg = o3d.pipelines.registration.registration_icp(
    pcd_src,
    pcd_tgt,
    threshold,
    np.eye(4),
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)
T_icp = reg.transformation
print("ICP Final Transformation:\n", T_icp)

P_final = (T_icp[:3, :3] @ P_transformed.T).T + T_icp[:3, 3]

steps = 100
pcd_anim = copy.deepcopy(pcd_src)
step_counter = 0
saved = False  

def animate(vis):
    global step_counter, pcd_anim, saved
    if step_counter > steps:
        if not saved:
            pcd_anim.paint_uniform_color([0,1,0])  
            vis.update_geometry(pcd_anim)
            vis.capture_screen_image("resultat_icp_final.png")  
            print("Image finale enregistrée : resultat_icp_final.png")
            saved = True
        return False

    alpha = step_counter / steps
    P_interp = P_transformed * (1 - alpha) + P_final * alpha
    pcd_anim.points = o3d.utility.Vector3dVector(P_interp)

    vis.update_geometry(pcd_anim)
    step_counter += 1
    time.sleep(0.03)
    return False

o3d.visualization.draw_geometries_with_animation_callback(
    [pcd_tgt, pcd_anim],
    animate
)
