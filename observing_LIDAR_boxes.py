import open3d as o3d
import numpy as np
import os
from tqdm import tqdm
import time
def boxes(path):
    caminho_arquivo = path
    with open(caminho_arquivo, 'r') as f:
        conteudo = f.read()
        
        # Unir todas as linhas em uma única string para simplificar a busca
        conteudo_unido = conteudo.replace("\n", " ").replace("  ", " ")

        # Encontrar a posição onde 'scores_3d' aparece
        inicio = conteudo_unido.find("'scores_3d': tensor(")
        if inicio != -1:
            # Encontrar a posição de abertura e fechamento dos colchetes []
            inicio_lista = conteudo_unido.find('[', inicio)
            fim_lista = conteudo_unido.find(']', inicio_lista)
            
            # Extrair a string que contém os números
            scores_str = conteudo_unido[inicio_lista+1:fim_lista]
            
            # Converter para lista de floats
            scores_list = [float(x) for x in scores_str.split(',') if x.strip()]
            
            
        # Encontrar a posição onde 'scores_3d' aparece
        inicio_2 = conteudo_unido.find("'boxes_3d': LiDARInstance3DBoxes(   tensor(")
        # print(conteudo_unido)
        if inicio_2 != -1:
            # Encontrar a posição de abertura e fechamento dos colchetes []
            inicio_lista = conteudo_unido.find('[[', inicio_2)
            fim_lista = conteudo_unido.find(']]', inicio_lista)
            
            # Extrair a string que contém os números
            bbox_str = conteudo_unido[inicio_lista+1:fim_lista+1]

            # Remover colchetes iniciais e finais
            cleaned_string = bbox_str.strip()[1:-1]
            cleaned_string = cleaned_string.replace('     ', '')
            list_of_lists = cleaned_string.split("],[")

            try:
                # # Converter cada lista interna em floats
                bbox_list = [[float(num) for num in sublist.split(",")] for sublist in list_of_lists]
            except:
                bbox_list = []
            # print(final_list)
        
        inicio_3 = conteudo_unido.find("'labels_3d': tensor(")
        if inicio_3 != -1:
            # Encontrar a posição de abertura e fechamento dos colchetes []
            inicio_lista = conteudo_unido.find('[', inicio_3)
            fim_lista = conteudo_unido.find(']', inicio_lista)
            
            # Extrair a string que contém os números
            label_str = conteudo_unido[inicio_lista+1:fim_lista]
            
            # Converter para lista de floats
            label_list = [int(x) for x in label_str.split(',') if x.strip()]


            out_list =[]
            for k in range(len(scores_list)):
                if scores_list[k]> 0.01:
                    try:
                        out_list.append(bbox_list[k])
                    except:
                        continue

    return out_list
def boxes_light(filename):
    # List to store the extracted data
    extracted_data = []

    # Read the file
    with open(filename, 'r') as file:
        for line in file:
            # Extract Position, Size, and Rotation using regular expressions
            split_data = []
            for part in line.split(':'):
                split_data.extend(part.split(','))
            try:
                if float(split_data[1])>=0.10:
                    extracted_data.append([int(split_data[13]),
                                           float(split_data[3]),
                                            float(split_data[4]),
                                            float(split_data[5]),
                                            float(split_data[7]),
                                            float(split_data[8]),
                                            float(split_data[9]),
                                            -float(split_data[11])])
                    
            except:
                continue
        # exit()
    return extracted_data

def draw_bounding_box(points, box_params, color = (1, 0, 0)):
    """Draw bounding box given the box parameters"""
    center = box_params[:3]
    size = box_params[3:6]
    rotation = box_params[6]

    # Create a bounding box in open3d
    bbox = o3d.geometry.OrientedBoundingBox(center, o3d.geometry.get_rotation_matrix_from_xyz([0, 0, rotation]), size)
    bbox.color = color  # Red color for bounding boxes
    return bbox

def find_closest_timestamp(target_timestamp, available_timestamps):
    closest_timestamp = min(available_timestamps, key=lambda x: abs(target_timestamp - x))
    return closest_timestamp



# name2 = '1718118287827761000'
# name =  '1718118287827761000'
# Load the point cloud from a PCD file




trajectory_config = {
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : False,
	"trajectory" : 
	[
		{
			"boundingbox_max" : [ 34.067462921142578, 160.10520935058594, 17.69294548034668 ],
			"boundingbox_min" : [ -82.221832275390625, -165.08709716796875, -3.6884143352508545 ],
			"field_of_view" : 60.0,
			"front" : [ 0.15043011093960623, -0.87831010570890511, 0.45380848375973537 ],
			"lookat" : [ -0.7275520827430586, -2.3006214267336107, 2.615340175320104 ],
			"up" : [ -0.069125807283045057, 0.44856382950563539, 0.8910735736327875 ],
			"zoom" : 0.039999999999999994
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}

# Visualize the point cloud and bounding boxes
# o3d.visualization.draw_geometries(geometries)

# Access the camera parameters from the trajectory config
camera_params = trajectory_config["trajectory"][0]

# Create a visualizer object
vis = o3d.visualization.Visualizer()
vis.create_window(visible = False)
opt = vis.get_render_option()
opt.point_size = 2.0  # Ajuste este valor conforme necessário

lidar_results_path = 'C:/Users/marped4128/Documents/Scripts/hi_drive_short_pcd/'
light_results_path = 'C:/Users/marped4128/Documents/Scripts/detections_Light3_sv3_cap10p_hi_drive_short/'
output_image_path = 'C:/Users/marped4128/Documents/Scripts/Light3_Noise_sv3_cap10p_hi_drive_short/'

lidar_files = sorted(os.listdir(lidar_results_path))
light_files = sorted(os.listdir(light_results_path))

lidar_timestamps = [int(f.split('.')[0]) for f in lidar_files]
light_timestamps = [int(f.split('.')[0]) for f in light_files]

obj_colors = {
    0 : (0, 0, 1),
    1 : (0, 0.5, 0),
    2 : (0.5, 1, 0.5),
    3 : (0.7, 0.4, 1),
    4 : (1, 0.5, 0),
    5 : (0, 1, 1),
    6 : (0.2, 0.2, 0.2),
    7 : (0.25, 0.1, 0),
    8 : (1, 0, 0),
    9 : (1.0, 1.0, 0),
}

# Car (Blue): (0, 0, 1)
# Truck (Dark Green): (0, 0.5, 0)
# Construction Vehicle (Light Green): (0.5, 1, 0.5)
# Bus (Light Purple): (0.7, 0.4, 1)
# Trailer (Yellow-Orange): (1, 0.5, 0)
# Barrier (Cyan): (0, 1, 1)
# Motorcycle (Dark Gray): (0.2, 0.2, 0.2)
# Bicycle (Dark-Orange): (0.25, 0.1, 0)
# Pedestrian (Red): (1, 0, 0)
# Traffic Cone (Yellow): (1.0, 1.0, 0)

def create_ring(radius, center, num_points=100):
    """Create a ring (circle) around a center point with a specified radius."""
    angles = np.linspace(0, 2 * np.pi, num_points)
    circle_points = np.array([[center[0] + radius * np.cos(a), center[1] + radius * np.sin(a), center[2]] for a in angles])
    
    # Convert points to a LineSet for visualization
    lines = [[i, (i+1) % num_points] for i in range(num_points)]  # Connect each point to the next
    ring = o3d.geometry.LineSet()
    ring.points = o3d.utility.Vector3dVector(circle_points)
    ring.lines = o3d.utility.Vector2iVector(lines)
    ring.paint_uniform_color([1.0, 1.0, 0.0])  # Set ring color to yellow
    return ring

def create_camera_diagonals(position, fov, depth, yaw_angle):
    """
    Cria duas diagonais que representam o campo de visão da câmera no plano XY, partindo do ponto 'position' até 'depth'.

    :param position: Lista ou array de 3 elementos representando o ponto de origem (x, y, z).
    :param fov: Ângulo do campo de visão em graus.
    :param depth: Distância máxima para as diagonais.
    :return: Um objeto LineSet representando as diagonais no plano XY.
    """
    # Convertendo o FOV de graus para radianos
    fov_rad = np.deg2rad(fov)
    
    # Ângulo máximo para as diagonais (metade do FOV)
    half_fov = fov_rad / 2
    
    # Coordenadas das diagonais no plano XY (apontando para frente no eixo X)
    # Diagonal esquerda (ângulo negativo)
    left_diagonal = [depth * np.sin(half_fov), depth * np.cos(half_fov), 0.0]
    
    # Diagonal direita (ângulo positivo)
    right_diagonal = [depth * np.sin(-half_fov), depth * np.cos(-half_fov), 0.0]
    
    yaw_rad = np.deg2rad(yaw_angle)
    rotation_matrix = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad), np.cos(yaw_rad), 0],
        [0, 0, 1]
    ])
    # Aplicar a rotação de yaw nas diagonais
    left_diagonal_rotated = np.dot(rotation_matrix, left_diagonal)
    right_diagonal_rotated = np.dot(rotation_matrix, right_diagonal)

    # Definir os pontos no plano XY
    points = np.array([
        position,               # Ponto de origem (0, 0, 0)
        left_diagonal_rotated,          # Ponto na diagonal esquerda
        right_diagonal_rotated          # Ponto na diagonal direita
    ])
    
    # Conectar os pontos: origem -> esquerda e origem -> direita
    lines = [
        [0, 1],  # Diagonal esquerda
        [0, 2]   # Diagonal direita
    ]
    
    # Criar o LineSet para as diagonais
    diagonals = o3d.geometry.LineSet()
    diagonals.points = o3d.utility.Vector3dVector(points)
    diagonals.lines = o3d.utility.Vector2iVector(lines)
    diagonals.colors = o3d.utility.Vector3dVector([[0, 1, 1], [0, 1, 1]])  # Cor vermelha para as diagonais
    return diagonals


# Example usage within your loop, assuming you want to create a ring around the origin [0, 0, 0]
radius = 50.0  # Define your radius
center = [-0.5, 0, 0]  # Replace with the actual center point if needed

# Parâmetros da câmera
camera_position = [0, 0, 0]        # Origem (0, 0, 0)
camera_fov = 55                    # Campo de visão de 90 graus
camera_depth = 50.0                # Distância máxima das diagonais
yaw_angle = 5

for lidar_file, lidar_timestamp in tqdm(zip(lidar_files[:5000], lidar_timestamps[:5000])):
    
    closest_light_timestamp = find_closest_timestamp(lidar_timestamp, light_timestamps)
    closest_light_timestamp = str(closest_light_timestamp)
    light_file = closest_light_timestamp + ".txt"




    pcd = o3d.io.read_point_cloud(lidar_results_path+ lidar_file[:-4]+'.pcd')

    # Bounding box parameters extracted from the provided file
    # boxs_lidar = np.array(boxes('C:/Users/marped4128/Documents/Scripts/ctag_sunny_results/'+ lidar_file))
    boxs_light = np.array(boxes_light(light_results_path+ light_file))
    
    # geometries = [pcd]
    vis.clear_geometries()
    vis.add_geometry(pcd)
    # for box_params in boxs_lidar:
    #     bbox = draw_bounding_box(pcd.points, box_params, color=(1,0,0))
    #     # geometries.append(bbox)
    #     vis.add_geometry(bbox)
    for box_params in boxs_light:
        bbox = draw_bounding_box(pcd.points, box_params[1:], color=obj_colors[box_params[0]])
        # geometries.append(bbox)
        vis.add_geometry(bbox)

    ring = create_ring(radius, center)
    vis.add_geometry(ring)

    # Adicionar o FOV da câmera
    diagonals = create_camera_diagonals(camera_position, camera_fov, camera_depth, yaw_angle)
    vis.add_geometry(diagonals)
    
    
    # Add geometries to the visualizer
    # for geometry in geometries:
    #     vis.add_geometry(geometry)

    # Get the view control and set the camera parameters from the JSON config
    view_ctl = vis.get_view_control()

    view_ctl.set_front(camera_params["front"])
    view_ctl.set_lookat(camera_params["lookat"])
    view_ctl.set_up(camera_params["up"])
    view_ctl.set_zoom(camera_params["zoom"])
    # time.sleep(0.1)  # Pequena pausa para garantir que a visualização esteja atualizada

    # Capture the view in an image
    vis.poll_events()
    vis.update_renderer()
    # vis.run()
    image = vis.capture_screen_float_buffer(False)

    # Convert the captured image buffer to an Open3D Image object
    image_np = (np.asarray(image) * 255).astype(np.uint8)
    image_o3d = o3d.geometry.Image(image_np)

    # Save the image
    o3d.io.write_image(output_image_path+lidar_file[:-4]+".jpg", image_o3d)
    # exit()
    # Close the visualizer window
vis.destroy_window()
    # input('Click')
