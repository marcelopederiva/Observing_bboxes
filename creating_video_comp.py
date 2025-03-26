import cv2
import os
import numpy as np
from tqdm import tqdm

def extract_timestamp(image_path):
    """
    Extrai o timestamp do nome do arquivo (removendo a extensão).
    """
    return int(os.path.splitext(os.path.basename(image_path))[0])

def get_images_with_timestamps(folder):
    """
    Obtém as imagens de uma pasta com os respectivos timestamps extraídos do nome.
    """
    images = [img for img in os.listdir(folder) if img.endswith((".png", ".jpg", ".jpeg"))]
    images_with_timestamps = {}
    for img in images:
        try:
            timestamp = extract_timestamp(img)
            images_with_timestamps[timestamp] = os.path.join(folder, img)
        except ValueError:
            print(f"Erro ao extrair timestamp do arquivo: {img}")
            continue
    return images_with_timestamps

def find_closest_timestamp(target_timestamp, available_timestamps):
    """
    Encontra o timestamp mais próximo em uma lista de timestamps disponíveis.
    """
    if not isinstance(target_timestamp, int):
        raise TypeError(f"target_timestamp deve ser um inteiro, mas recebeu {type(target_timestamp)}: {target_timestamp}")
    if not available_timestamps:
        raise ValueError("available_timestamps está vazio.")
    closest_timestamp = min(available_timestamps, key=lambda x: abs(target_timestamp - x))
    return closest_timestamp

def match_heights(img1, img2):
    """
    Ajusta as alturas de duas imagens, adicionando bordas pretas à menor.
    """
    h1, w1 = img1.shape[:2]
    h2, w2 = img2.shape[:2]

    if h1 > h2:
        # Adiciona bordas pretas à img2
        delta_h = h1 - h2
        top = delta_h // 2
        bottom = delta_h - top
        img2 = cv2.copyMakeBorder(img2, top, bottom, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])
    elif h2 > h1:
        # Adiciona bordas pretas à img1
        delta_h = h2 - h1
        top = delta_h // 2
        bottom = delta_h - top
        img1 = cv2.copyMakeBorder(img1, top, bottom, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])

    return img1, img2

def add_title(image, title, width):
    """
    Adiciona um título no topo da imagem.
    """
    # Define o espaço extra para o título
    title_height = 50
    title_space = np.zeros((title_height, width, 3), dtype=np.uint8)

    # Define o texto
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    font_thickness = 2
    text_size = cv2.getTextSize(title, font, font_scale, font_thickness)[0]
    text_x = (width - text_size[0]) // 2
    text_y = (title_height + text_size[1]) // 2

    # Adiciona o texto na parte superior
    cv2.putText(title_space, title, (text_x, text_y), font, font_scale, (255, 255, 255), font_thickness, cv2.LINE_AA)

    # Combina o espaço do título com a imagem original
    image_with_title = np.vstack((title_space, image))

    return image_with_title

# Pastas das imagens
folder_1 = 'C:/Users/marped4128/Documents/Scripts/Light3_1swp_hi_drive_short'
folder_2 = 'C:/Users/marped4128/Documents/Scripts/Light3_Noise3_hi_drive_short'

# Nome do vídeo de saída
video_name = 'video_side_by_side.mp4'

# Obtém as imagens com timestamps
images_with_timestamps1 = get_images_with_timestamps(folder_1)
images_with_timestamps2 = get_images_with_timestamps(folder_2)

# Lista de timestamps disponíveis em cada pasta
timestamps1 = sorted(images_with_timestamps1.keys())
timestamps2 = sorted(images_with_timestamps2.keys())

# Verifica se há timestamps disponíveis
if not timestamps1 or not timestamps2:
    raise ValueError("Uma das pastas não contém imagens válidas com timestamps.")

# Inicializa o video writer
video = None

# Define a resolução desejada para o vídeo
target_width = 1280  # Reduzindo a largura para 1280 pixels
target_height = 720 # Reduzindo a altura para 720 pixels

# Loop para processar os timestamps da primeira pasta
for timestamp in tqdm(timestamps1[:7000]):
    # Verifique se o timestamp é válido
    if not isinstance(timestamp, int):
        print(f"Timestamp inválido encontrado: {timestamp}")
        continue

    # Encontra a imagem mais próxima na segunda pasta
    closest_timestamp = find_closest_timestamp(timestamp, timestamps2)

    # Carrega as imagens correspondentes
    img1 = cv2.imread(images_with_timestamps1[timestamp])
    img2 = cv2.imread(images_with_timestamps2[closest_timestamp])

    # Verifica se as imagens foram carregadas corretamente
    if img1 is None or img2 is None:
        print(f"Erro ao carregar imagens para timestamp {timestamp}: {images_with_timestamps1.get(timestamp)} ou {images_with_timestamps2.get(closest_timestamp)}")
        continue

    # Redimensiona as imagens para a resolução desejada
    img1 = cv2.resize(img1, (target_width, target_height))
    img2 = cv2.resize(img2, (target_width, target_height))

    # Ajusta as alturas das imagens para serem iguais
    img1, img2 = match_heights(img1, img2)

    # Adiciona títulos às imagens
    img1 = add_title(img1, "Light 3 Original", img1.shape[1])
    img2 = add_title(img2, "Light 3 Noise", img2.shape[1])

    # Combina as imagens lado a lado
    combined_frame = np.hstack((img1, img2))

    # Inicializa o video writer se ainda não estiver inicializado
    if video is None:
        height, width, _ = combined_frame.shape
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video = cv2.VideoWriter(video_name, fourcc, 24.0, (width, height))

    # Adiciona o frame ao vídeo
    video.write(combined_frame)

# Libera o video writer
if video is not None:
    video.release()

print(f"Vídeo salvo como {video_name}")