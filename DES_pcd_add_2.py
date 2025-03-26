import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

def density_equalization_sampling(points, dnear=20, dfar=70, s1=0.2, yaw_angles=720, radius_gaps=50, distance_thresh=5):
    # Dividir a cena em diferentes ângulos de yaw e faixas de raio
    yaw_step = 2 * np.pi / yaw_angles
    radius_step = (dfar - dnear) / radius_gaps
    
    # Criar volumes (yaw x raio)
    volumes = [[[] for _ in range(radius_gaps)] for _ in range(yaw_angles)]
    
    # Classificar os pontos em volumes baseados em yaw e raio
    for point in points:
        x, y, z, intensity = point
        distance = np.sqrt(x**2 + y**2)
        if dnear <= distance <= dfar:
            yaw = np.arctan2(y, x) % (2 * np.pi)
            yaw_idx = int(yaw // yaw_step)
            radius_idx = min(int((distance - dnear) // radius_step), radius_gaps - 1)
            volumes[yaw_idx][radius_idx].append(point)
    
    sampled_points = []
    created_points = []
    
    # Iterar por cada volume e adicionar pontos sintéticos, se necessário
    for yaw_idx in range(yaw_angles):
        for radius_idx in range(radius_gaps):
            region = volumes[yaw_idx][radius_idx]
            if len(region) == 0:
                continue
            
            # Verificar se a distância média dos pontos é maior que o limite
            region_array = np.array(region)
            mean_point = np.mean(region_array, axis=0)
            random_offset = np.random.uniform(-s1, s1, size=3)
            new_point = mean_point[:3] + random_offset
            new_point[2:3] = mean_point[2:3]
            new_point = np.append(new_point, mean_point[3])  # Manter a intensidade original
            created_points.append(new_point)
            
            sampled_points.extend(region)
    
    # Converter os pontos amostrados e criados em um array
    sampled_points = np.array(sampled_points)
    created_points = np.array(created_points)
    
    return sampled_points, created_points

# Exemplo de uso
if __name__ == "__main__":
    # Carregar dados de nuvem de pontos
    pcd = o3d.io.read_point_cloud("C:/Users/marped4128/Documents/Scripts/hi_drive_short_pcd/1726226783150127872.pcd")
    points = np.asarray(pcd.points)
    
    # Adicionar uma intensidade aleatória para cada ponto (se não existir)
    if points.shape[1] == 3:
        intensities = np.random.uniform(0, 1, size=(points.shape[0], 1))
        points_ = np.hstack((points, intensities))
    
    sampled_points, created_points = density_equalization_sampling(points_)
    # print(created_points)
    # exit()
    sampled_points_array = np.array(sampled_points)
    created_points_array = np.array(created_points)

    original_pcd = o3d.geometry.PointCloud()
    original_pcd.points = o3d.utility.Vector3dVector(points)
    original_pcd.paint_uniform_color([0, 0, 1])  # Cor azul para os pontos originais

    created_pcd = o3d.geometry.PointCloud()
    created_pcd.points = o3d.utility.Vector3dVector(created_points_array[:, :3])
    created_pcd.paint_uniform_color([1, 0, 0])  # Cor vermelha para os pontos criados



    # Criação da visualização com opções de controle de tamanho
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Pontos Originais e Criados")
    vis.add_geometry(original_pcd)
    vis.add_geometry(created_pcd)

    # Controlar o tamanho dos pontos
    opt = vis.get_render_option()
    opt.point_size = 2.0  # Ajuste este valor para diminuir/aumentar o tamanho dos pontos

    vis.run()
    vis.destroy_window()


    # # Visualizar os pontos originais e os pontos criados juntos
    # o3d.visualization.draw_geometries([original_pcd, created_pcd], window_name="Pontos Originais e Criados")