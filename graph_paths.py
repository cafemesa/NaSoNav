
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# Load the PGM image
pgm_image = mpimg.imread("maps/map.pgm")

# Determine the extent based on the size of the image and the pixel scale
pixel_scale = 0.05  # 0.05 meters per pixel
image_height, image_width = pgm_image.shape
x_center = (image_width / 2) * pixel_scale
y_center = (image_height / 2) * pixel_scale
xmin = -x_center
xmax = x_center
ymin = -y_center
ymax = y_center

# Create a figure and axis
fig, ax = plt.subplots()

# Display the PGM image as the background with the correct scaling
ax.imshow(pgm_image, cmap='gray', extent=[xmin, xmax, ymin, ymax])

# Set the desired limits for the plot
ax.set_xlim([-20, 7])
ax.set_ylim([-8, 3])

x_offset = -12.091963
y_offset = -5.868032



# Función para graficar un archivo CSV
def graficar_csv(nombre_archivo, color, i):
    # Leer el archivo CSV
    df = pd.read_csv(nombre_archivo)
    
    # Extraer las columnas X e Y
    x = df['x'] + x_offset
    y = df['y'] + y_offset
    
    # Graficar los puntos with the specified color
    ax.scatter(x, y, label=labels[i], color=color, s=5)

labels = ["GT", "PUA", "SF", "ORCA", "SACADRL"]

# Solicitar al usuario la cantidad de archivos a procesar
num_archivos = ["annotations/model/scene6_1.5_0_robotPositionsFileName.csv", "annotations/tests/scene6_1.5_1_unaware_robotPositionsFileName.csv", "annotations/tests/scene6_1.5_1_sf_robotPositionsFileName.csv", "annotations/tests/scene6_1.5_1_rvo_robotPositionsFileName.csv", "annotations/tests/scene6_1.5_1_sacadrl_robotPositionsFileName.csv"]
# Crear una lista de colores para asignar a cada archivo
colores = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

# Iterar a través de los archivos
for i in range(len(num_archivos)):
    color = colores[i % len(colores)]  # Ciclar a través de los colores
    graficar_csv(num_archivos[i], color, i)

# Configure the plot
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.legend()
ax.set_title('Gráfico de coordenadas 2D')

# Show the plot with the background image
plt.show()
