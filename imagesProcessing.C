// Programa de procesamiento de imágenes en C para principiantes en Linux.
// QUÉ: Procesa imágenes PNG (escala de grises o RGB) usando matrices, con
// soporte para carga, visualización, guardado y ajuste de brillo concurrente.
// CÓMO: Usa stb_image.h para cargar PNG y stb_image_write.h para guardar PNG,
// con hilos POSIX (pthread) para el procesamiento paralelo del brillo.
// POR QUÉ: Diseñado para enseñar manejo de matrices, concurrencia y gestión de
// memoria en C, manteniendo simplicidad y robustez para principiantes.
// Dependencias: Descarga stb_image.h y stb_image_write.h desde
// https://github.com/nothings/stb
//   wget https://raw.githubusercontent.com/nothings/stb/master/stb_image.h
//   wget
//   https://raw.githubusercontent.com/nothings/stb/master/stb_image_write.h
//
// Compilar: gcc -o img img_base.c -pthread -lm
// Ejecutar: ./img [ruta_imagen.png]

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// QUÉ: Incluir bibliotecas stb para cargar y guardar imágenes PNG.
// CÓMO: stb_image.h lee PNG/JPG a memoria; stb_image_write.h escribe PNG.
// POR QUÉ: Son bibliotecas de un solo archivo, simples y sin dependencias
// externas.
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// QUÉ: Estructura para almacenar la imagen (ancho, alto, canales, píxeles).
// CÓMO: Usa matriz 3D para píxeles (alto x ancho x canales), donde canales es
// 1 (grises) o 3 (RGB). Píxeles son unsigned char (0-255).
// POR QUÉ: Permite manejar tanto grises como color, con memoria dinámica para
// flexibilidad y evitar desperdicio.
typedef struct {
  int ancho;                // Ancho de la imagen en píxeles
  int alto;                 // Alto de la imagen en píxeles
  int canales;              // 1 (escala de grises) o 3 (RGB)
  unsigned char ***pixeles; // Matriz 3D: [alto][ancho][canales]
} ImagenInfo;
typedef struct {
  unsigned char ***pixelelesOrigen;
  unsigned char ***pixelesDestino;
  float **kernel;
  int tamKernel;
  int inicio;
  int final;
  int ancho;
  int alto;
  int canales;
} ConvolucionArgs;

// QUÉ: Número global de hilos para operaciones concurrentes.
static int GLOBAL_NUM_THREADS = 2;

typedef struct {
    ImagenInfo* imagenOriginal;
    ImagenInfo* imagenRotada;
    double angleDegrees;
    int inicioY;
    int finalY;
} RotationArgs;

typedef struct {
    ImagenInfo* imagenOriginal;
    ImagenInfo* imagenBordes;
    int inicioY;
    int finalY;
} SobelArgs;

// QUÉ: Liberar memoria asignada para la imagen.
// CÓMO: Libera cada fila y canal de la matriz 3D, luego el arreglo de filas y
// reinicia la estructura.
// POR QUÉ: Evita fugas de memoria, esencial en C para manejar recursos
// manualmente.
void liberarImagen(ImagenInfo *info) {
  if (info->pixeles) {
    for (int y = 0; y < info->alto; y++) {
      for (int x = 0; x < info->ancho; x++) {
        free(info->pixeles[y][x]); // Liberar canales por píxel
      }
      free(info->pixeles[y]); // Liberar fila
    }
    free(info->pixeles); // Liberar arreglo de filas
    info->pixeles = NULL;
  }
  info->ancho = 0;
  info->alto = 0;
  info->canales = 0;
}

float **generarKernel(int tamKernel, float sigma) {
  float **kernel = (float **)malloc(tamKernel * sizeof(float *));
  for (int i = 0; i < tamKernel; i++) {
    kernel[i] = (float *)malloc(tamKernel * sizeof(float));
  }

  float suma = 0.0f;
  int centro = tamKernel / 2;

  for (int y = 0; y < tamKernel; y++) {
    for (int x = 0; x < tamKernel; x++) {
      int dy = y - centro;
      int dx = x - centro;
      kernel[y][x] = exp(-(dx * dx + dy * dy) / (2 * sigma * sigma)) /
                     (2 * M_PI * sigma * sigma);
      suma += kernel[y][x];
    }
  }

  // Normalizar el kernel
  for (int y = 0; y < tamKernel; y++) {
    for (int x = 0; x < tamKernel; x++) {
      kernel[y][x] /= suma;
    }
  }

  return kernel;
}

void liberarKernel(float **kernel, int tamKernel) {
  for (int i = 0; i < tamKernel; i++) {
    free(kernel[i]);
  }
  free(kernel);
}

void *aplicarConvolucion(void *args) {
  ConvolucionArgs *cArgs = (ConvolucionArgs *)args;
  int tamKernel = cArgs->tamKernel;
  int centro = tamKernel / 2;

  for (int y = cArgs->inicio; y < cArgs->final; y++) {
    for (int x = 0; x < cArgs->ancho; x++) {
      for (int c = 0; c < cArgs->canales; c++) {
        float valor = 0.0f;
        for (int ky = 0; ky < tamKernel; ky++) {
          for (int kx = 0; kx < tamKernel; kx++) {
            int imgY = y + ky - centro;
            int imgX = x + kx - centro;

            if (imgY >= 0 && imgY < cArgs->alto && imgX >= 0 &&
                imgX < cArgs->ancho) {
              valor +=
                  cArgs->pixelelesOrigen[imgY][imgX][c] * cArgs->kernel[ky][kx];
            }
          }
        }
        if (valor < 0)
          valor = 0;
        if (valor > 255)
          valor = 255;
        cArgs->pixelesDestino[y][x][c] = (unsigned char)(valor);
      }
    }
  }
  return NULL; // ← Soluciona el warning
}

void aplicarDesenfoque(ImagenInfo *info, int numHilos, int tamKernel,
                       float sigma) {
  if (!info->pixeles) {
    printf("No hay imagen cargada para procesar.\n");
    return;
  }

  printf(
      "Aplicando desenfoque con kernel %dx%d y sigma %.2f usando %d hilos...\n",
      tamKernel, tamKernel, sigma, numHilos);

  float **kernel = generarKernel(tamKernel, sigma);
  unsigned char ***pixelesDestino =
      (unsigned char ***)malloc(info->alto * sizeof(unsigned char **));
  for (int y = 0; y < info->alto; y++) {
    pixelesDestino[y] =
        (unsigned char **)malloc(info->ancho * sizeof(unsigned char *));
    for (int x = 0; x < info->ancho; x++) {
      pixelesDestino[y][x] =
          (unsigned char *)malloc(info->canales * sizeof(unsigned char));
    }
  }

  pthread_t hilos[numHilos];
  ConvolucionArgs args[numHilos];
  int filasPorHilo = info->alto / numHilos;
  for (int i = 0; i < numHilos; i++) {
    args[i].pixelelesOrigen = info->pixeles;
    args[i].pixelesDestino = pixelesDestino;
    args[i].kernel = kernel;
    args[i].tamKernel = tamKernel;
    args[i].inicio = i * filasPorHilo;
    args[i].final = (i == numHilos - 1) ? info->alto : (i + 1) * filasPorHilo;
    args[i].ancho = info->ancho;
    args[i].alto = info->alto;
    args[i].canales = info->canales;
    pthread_create(&hilos[i], NULL, aplicarConvolucion, &args[i]);
  }

  for (int i = 0; i < numHilos; i++) {
    pthread_join(hilos[i], NULL);
  }

  for (int y = 0; y < info->alto; y++) {
    for (int x = 0; x < info->ancho; x++) {
      for (int c = 0; c < info->canales; c++) {
        info->pixeles[y][x][c] = pixelesDestino[y][x][c];
      }
    }
  }

  for (int y = 0; y < info->alto; y++) {
    for (int x = 0; x < info->ancho; x++) {
      free(pixelesDestino[y][x]);
    }
    free(pixelesDestino[y]);
  }
  free(pixelesDestino);

  liberarKernel(kernel, tamKernel);
  printf("Desenfoque aplicado.\n");
}

#include <math.h>

void calculateRotatedDimensions(ImagenInfo *originalinfo, double angleDegrees,
                                ImagenInfo *rotatedInfo) {
  double angleRad = angleDegrees * M_PI / 180.0;
  double cosAngle = fabs(cos(angleRad));
  double sinAngle = fabs(sin(angleRad));

  // New bounding box dimensions
  rotatedInfo->ancho =
      (int)ceil(originalinfo->ancho * cosAngle + originalinfo->alto * sinAngle);
  rotatedInfo->alto =
      (int)ceil(originalinfo->ancho * sinAngle + originalinfo->alto * cosAngle);
}

int allocateRotatedImage(ImagenInfo *originalInfo, ImagenInfo *rotatedInfo,
                         double angleDegrees) {
  // Calculate new dimensions
  calculateRotatedDimensions(originalInfo, angleDegrees, rotatedInfo);

  // Keep same number of channels
  rotatedInfo->canales = originalInfo->canales;

  // Allocate memory for the new dimensions
  rotatedInfo->pixeles =
      (unsigned char ***)malloc(rotatedInfo->alto * sizeof(unsigned char **));
  if (!rotatedInfo->pixeles) {
    return 0; // Memory allocation failed
  }

  for (int y = 0; y < rotatedInfo->alto; y++) {
    rotatedInfo->pixeles[y] =
        (unsigned char **)malloc(rotatedInfo->ancho * sizeof(unsigned char *));
    if (!rotatedInfo->pixeles[y]) {
      // Clean up previously allocated memory
      for (int i = 0; i < y; i++) {
        for (int x = 0; x < rotatedInfo->ancho; x++) {
          free(rotatedInfo->pixeles[i][x]);
        }
        free(rotatedInfo->pixeles[i]);
      }
      free(rotatedInfo->pixeles);
      return 0;
    }

    for (int x = 0; x < rotatedInfo->ancho; x++) {
      rotatedInfo->pixeles[y][x] =
          (unsigned char *)malloc(rotatedInfo->canales * sizeof(unsigned char));
      if (!rotatedInfo->pixeles[y][x]) {
        // Clean up previously allocated memory
        for (int i = 0; i <= y; i++) {
          int maxX = (i == y) ? x : rotatedInfo->ancho;
          for (int j = 0; j < maxX; j++) {
            free(rotatedInfo->pixeles[i][j]);
          }
          free(rotatedInfo->pixeles[i]);
        }
        free(rotatedInfo->pixeles);
        return 0;
      }

      // Initialize with background color (e.g., black or white)
      for (int c = 0; c < rotatedInfo->canales; c++) {
        rotatedInfo->pixeles[y][x][c] = 0; // Black background
      }
    }
  }

  return 1; // Success
}

void* rotacionHilo(void* args) {
    RotationArgs* rArgs = (RotationArgs*)args;
    
    double angleRad = rArgs->angleDegrees * M_PI / 180.0;
    double cosAngle = cos(angleRad);
    double sinAngle = sin(angleRad);
    
    // Calculate center points
    double originalCenterX = rArgs->imagenOriginal->ancho / 2.0;
    double originalCenterY = rArgs->imagenOriginal->alto / 2.0;
    double newCenterX = rArgs->imagenRotada->ancho / 2.0;
    double newCenterY = rArgs->imagenRotada->alto / 2.0;
    
    // Process assigned rows
    for (int newY = rArgs->inicioY; newY < rArgs->finalY; newY++) {
        for (int newX = 0; newX < rArgs->imagenRotada->ancho; newX++) {
            // Translate to center
            double translatedX = newX - newCenterX;
            double translatedY = newY - newCenterY;
            
            // Apply inverse rotation
            double originalX = translatedX * cosAngle + translatedY * sinAngle + originalCenterX;
            double originalY = -translatedX * sinAngle + translatedY * cosAngle + originalCenterY;
            
            // Check if the source pixel is within bounds
            int srcX = (int)round(originalX);
            int srcY = (int)round(originalY);
            
            if (srcX >= 0 && srcX < rArgs->imagenOriginal->ancho && 
                srcY >= 0 && srcY < rArgs->imagenOriginal->alto) {
                // Copy all channels
                for (int c = 0; c < rArgs->imagenOriginal->canales; c++) {
                    rArgs->imagenRotada->pixeles[newY][newX][c] = 
                        rArgs->imagenOriginal->pixeles[srcY][srcX][c];
                }
            }
            // If outside bounds, keep the initialized background color (already set to 0)
        }
    }
    
    return NULL;
}

// QUÉ: Rota la imagen en un ángulo dado (e.g., 90°, 180°, 270° o arbitrario).
// CÓMO: Calcula nuevas coordenadas usando matrices de transformación (e.g., x'
// = xcosθ - ysinθ, y' = xsinθ + ycosθ). Usa interpolación bilineal para píxeles
// no enteros. Crea una nueva matriz para la imagen rotada y libera la antigua.
// POR QUÉ: Involucra transformaciones geométricas matriciales y manejo de
// dimensiones cambiantes. Parámetros: ImagenInfo* info, float angulo (en
// grados).
// Concurrencia: Paraleliza el cálculo de píxeles en la nueva matriz
// dividiendo por filas o bloques.
void rotarImagen(ImagenInfo *info, float angulo) {
  if (!info->pixeles) {
    printf("No image loaded to rotate.\n");
    return;
  }

  ImagenInfo rotatedInfo = {0};

  // Allocate memory for rotated image
  if (!allocateRotatedImage(info, &rotatedInfo, angulo)) {
    printf("Failed to allocate memory for rotated image.\n");
    return;
  }

  // QUÉ: Usar múltiples hilos para acelerar la rotación.
  // CÓMO: Divide las filas de la imagen rotada entre GLOBAL_NUM_THREADS hilos.
  // POR QUÉ: Paralelizar mejora el rendimiento en imágenes grandes.
  pthread_t hilos[GLOBAL_NUM_THREADS];
  RotationArgs args[GLOBAL_NUM_THREADS];
  int filasPorHilo = (int)ceil((double)rotatedInfo.alto / GLOBAL_NUM_THREADS);

  printf("Rotando imagen usando %d hilos...\n", GLOBAL_NUM_THREADS);

  // Crear y lanzar hilos
  for (int i = 0; i < GLOBAL_NUM_THREADS; i++) {
    args[i].imagenOriginal = info;
    args[i].imagenRotada = &rotatedInfo;
    args[i].angleDegrees = angulo;
    args[i].inicioY = i * filasPorHilo;
    args[i].finalY = (i == GLOBAL_NUM_THREADS - 1) ? rotatedInfo.alto : (i + 1) * filasPorHilo;

    if (pthread_create(&hilos[i], NULL, rotacionHilo, &args[i]) != 0) {
      fprintf(stderr, "Error al crear hilo %d para rotación\n", i);
      break;
    }
  }

  // Esperar a que todos los hilos terminen
  for (int i = 0; i < GLOBAL_NUM_THREADS; i++) {
    pthread_join(hilos[i], NULL);
  }

  // Replace original image with rotated image
  liberarImagen(info);
  *info = rotatedInfo;

  printf("Imagen rotada %.1f grados usando %d hilos. Nuevas dimensiones: %dx%d\n", 
         angulo, GLOBAL_NUM_THREADS, info->ancho, info->alto);
}

// QUÉ: Liberar memoria asignada para la imagen.
// CÓMO: Libera cada fila y canal de la matriz 3D, luego el arreglo de filas y
// reinicia la estructura.
// POR QUÉ: Evita fugas de memoria, esencial en C para manejar recursos
// manualmente.

// QUÉ: Cargar una imagen PNG desde un archivo.
// CÓMO: Usa stbi_load para leer el archivo, detecta canales (1 o 3), y
// convierte los datos a una matriz 3D (alto x ancho x canales). POR QUÉ: La
// matriz 3D es intuitiva para principiantes y permite procesar píxeles y
// canales individualmente.
int cargarImagen(const char *ruta, ImagenInfo *info) {
  int canales;
  // QUÉ: Cargar imagen con formato original (0 canales = usar formato nativo).
  // CÓMO: stbi_load lee el archivo y llena ancho, alto y canales.
  // POR QUÉ: Respetar el formato original asegura que grises o RGB se
  // mantengan.
  unsigned char *datos =
      stbi_load(ruta, &info->ancho, &info->alto, &canales, 0);
  if (!datos) {
    fprintf(stderr, "Error al cargar imagen: %s\n", ruta);
    return 0;
  }
  info->canales = (canales == 1 || canales == 3) ? canales : 1; // Forzar 1 o 3

  // QUÉ: Asignar memoria para matriz 3D.
  // CÓMO: Asignar alto filas, luego ancho columnas por fila, luego canales por
  // píxel. POR QUÉ: Estructura clara y flexible para grises (1 canal) o RGB (3
  // canales).
  info->pixeles =
      (unsigned char ***)malloc(info->alto * sizeof(unsigned char **));
  if (!info->pixeles) {
    fprintf(stderr, "Error de memoria al asignar filas\n");
    stbi_image_free(datos);
    return 0;
  }
  for (int y = 0; y < info->alto; y++) {
    info->pixeles[y] =
        (unsigned char **)malloc(info->ancho * sizeof(unsigned char *));
    if (!info->pixeles[y]) {
      fprintf(stderr, "Error de memoria al asignar columnas\n");
      liberarImagen(info);
      stbi_image_free(datos);
      return 0;
    }
    for (int x = 0; x < info->ancho; x++) {
      info->pixeles[y][x] =
          (unsigned char *)malloc(info->canales * sizeof(unsigned char));
      if (!info->pixeles[y][x]) {
        fprintf(stderr, "Error de memoria al asignar canales\n");
        liberarImagen(info);
        stbi_image_free(datos);
        return 0;
      }
      // Copiar píxeles a matriz 3D
      for (int c = 0; c < info->canales; c++) {
        info->pixeles[y][x][c] =
            datos[(y * info->ancho + x) * info->canales + c];
      }
    }
  }

  stbi_image_free(datos); // Liberar buffer de stb
  printf("Imagen cargada: %dx%d, %d canales (%s)\n", info->ancho, info->alto,
         info->canales, info->canales == 1 ? "grises" : "RGB");
  return 1;
}

// QUÉ: Mostrar la matriz de píxeles (primeras 10 filas).
// CÓMO: Imprime los valores de los píxeles, agrupando canales por píxel (grises
// o RGB). POR QUÉ: Ayuda a visualizar la matriz para entender la estructura de
// datos.
void mostrarMatriz(const ImagenInfo *info) {
  if (!info->pixeles) {
    printf("No hay imagen cargada.\n");
    return;
  }
  printf("Matriz de la imagen (primeras 10 filas):\n");
  for (int y = 0; y < info->alto && y < 10; y++) {
    for (int x = 0; x < info->ancho; x++) {
      if (info->canales == 1) {
        printf("%3u ", info->pixeles[y][x][0]); // Escala de grises
      } else {
        printf("(%3u,%3u,%3u) ", info->pixeles[y][x][0], info->pixeles[y][x][1],
               info->pixeles[y][x][2]); // RGB
      }
    }
    printf("\n");
  }
  if (info->alto > 10) {
    printf("... (más filas)\n");
  }
}

// QUÉ: Guardar la matriz como PNG (grises o RGB).
// CÓMO: Aplana la matriz 3D a 1D y usa stbi_write_png con el número de canales
// correcto. POR QUÉ: Respeta el formato original (grises o RGB) para
// consistencia.
int guardarPNG(const ImagenInfo *info, const char *rutaSalida) {
  if (!info->pixeles) {
    fprintf(stderr, "No hay imagen para guardar.\n");
    return 0;
  }

  // QUÉ: Aplanar matriz 3D a 1D para stb.
  // CÓMO: Copia píxeles en orden [y][x][c] a un arreglo plano.
  // POR QUÉ: stb_write_png requiere datos contiguos.
  unsigned char *datos1D =
      (unsigned char *)malloc(info->ancho * info->alto * info->canales);
  if (!datos1D) {
    fprintf(stderr, "Error de memoria al aplanar imagen\n");
    return 0;
  }
  for (int y = 0; y < info->alto; y++) {
    for (int x = 0; x < info->ancho; x++) {
      for (int c = 0; c < info->canales; c++) {
        datos1D[(y * info->ancho + x) * info->canales + c] =
            info->pixeles[y][x][c];
      }
    }
  }

  // QUÉ: Guardar como PNG.
  // CÓMO: Usa stbi_write_png con los canales de la imagen original.
  // POR QUÉ: Mantiene el formato (grises o RGB) de la entrada.
  int resultado =
      stbi_write_png(rutaSalida, info->ancho, info->alto, info->canales,
                     datos1D, info->ancho * info->canales);
  free(datos1D);
  if (resultado) {
    printf("Imagen guardada en: %s (%s)\n", rutaSalida,
           info->canales == 1 ? "grises" : "RGB");
    return 1;
  } else {
    fprintf(stderr, "Error al guardar PNG: %s\n", rutaSalida);
    return 0;
  }
}

// QUÉ: Estructura para pasar datos al hilo de ajuste de brillo.
// CÓMO: Contiene matriz, rango de filas, ancho, canales y delta de brillo.
// POR QUÉ: Los hilos necesitan datos específicos para procesar en paralelo.
typedef struct {
  unsigned char ***pixeles;
  int inicio;
  int fin;
  int ancho;
  int canales;
  int delta;
} BrilloArgs;

// QUÉ: Ajustar brillo en un rango de filas (para hilos).
// CÓMO: Suma delta a cada canal de cada píxel, con clamp entre 0-255.
// POR QUÉ: Procesa píxeles en paralelo para demostrar concurrencia.
void *ajustarBrilloHilo(void *args) {
  BrilloArgs *bArgs = (BrilloArgs *)args;
  for (int y = bArgs->inicio; y < bArgs->fin; y++) {
    for (int x = 0; x < bArgs->ancho; x++) {
      for (int c = 0; c < bArgs->canales; c++) {
        int nuevoValor = bArgs->pixeles[y][x][c] + bArgs->delta;
        bArgs->pixeles[y][x][c] =
            (unsigned char)(nuevoValor < 0
                                ? 0
                                : (nuevoValor > 255 ? 255 : nuevoValor));
      }
    }
  }
  return NULL;
}

// QUÉ: Ajustar brillo de la imagen usando múltiples hilos.
// CÓMO: Divide las filas entre 2 hilos, pasa argumentos y espera con join.
// POR QUÉ: Usa concurrencia para acelerar el procesamiento y enseñar hilos.
void ajustarBrilloConcurrente(ImagenInfo *info, int delta) {
  if (!info->pixeles) {
    printf("No hay imagen cargada.\n");
    return;
  }

  const int numHilos = 2; // QUÉ: Número fijo de hilos para simplicidad.
  pthread_t hilos[numHilos];
  BrilloArgs args[numHilos];
  int filasPorHilo = (int)ceil((double)info->alto / numHilos);

  // QUÉ: Configurar y lanzar hilos.
  // CÓMO: Asigna rangos de filas a cada hilo y pasa datos.
  // POR QUÉ: Divide el trabajo para procesar en paralelo.
  for (int i = 0; i < numHilos; i++) {
    args[i].pixeles = info->pixeles;
    args[i].inicio = i * filasPorHilo;
    args[i].fin = (i + 1) * filasPorHilo < info->alto ? (i + 1) * filasPorHilo
                                                      : info->alto;
    args[i].ancho = info->ancho;
    args[i].canales = info->canales;
    args[i].delta = delta;
    if (pthread_create(&hilos[i], NULL, ajustarBrilloHilo, &args[i]) != 0) {
      fprintf(stderr, "Error al crear hilo %d\n", i);
      return;
    }
  }

  // QUÉ: Esperar a que los hilos terminen.
  // CÓMO: Usa pthread_join para sincronizar.
  // POR QUÉ: Garantiza que todos los píxeles se procesen antes de continuar.
  for (int i = 0; i < numHilos; i++) {
    pthread_join(hilos[i], NULL);
  }
  printf("Brillo ajustado concurrentemente con %d hilos (%s).\n", numHilos,
         info->canales == 1 ? "grises" : "RGB");
}

void* aplicarSobelHilo(void* arg) {
    SobelArgs* args = (SobelArgs*)arg;
    ImagenInfo* src = args->imagenOriginal;
    ImagenInfo* dst = args->imagenBordes;
    int ancho = src->ancho;
    int canales = src->canales;
    int Gx[3][3] = {
        {-1, 0, 1},
        {-2, 0, 2},
        {-1, 0, 1}
    };
    int Gy[3][3] = {
        {-1, -2, -1},
        { 0,  0,  0},
        { 1,  2,  1}
    };
    for (int y = args->inicioY; y < args->finalY; y++) {
        for (int x = 1; x < ancho - 1; x++) {
            int gx = 0, gy = 0;
            for (int ky = -1; ky <= 1; ky++) {
                for (int kx = -1; kx <= 1; kx++) {
                    int pixel;
                    if (canales == 1) {
                        pixel = src->pixeles[y + ky][x + kx][0];
                    } else {
                        pixel = (src->pixeles[y + ky][x + kx][0] + src->pixeles[y + ky][x + kx][1] + src->pixeles[y + ky][x + kx][2]) / 3;
                    }
                    gx += pixel * Gx[ky + 1][kx + 1];
                    gy += pixel * Gy[ky + 1][kx + 1];
                }
            }
            int mag = (int)sqrt(gx * gx + gy * gy);
            if (mag > 255) mag = 255;
            if (mag < 0) mag = 0;
            dst->pixeles[y][x][0] = (unsigned char)mag;
            if (dst->canales == 3) {
                dst->pixeles[y][x][1] = (unsigned char)mag;
                dst->pixeles[y][x][2] = (unsigned char)mag;
            }
        }
    }
    return NULL;
}

void detectarBordes(ImagenInfo* imagen) {
    if (!imagen || !imagen->pixeles) {
        printf("No hay imagen cargada.\n");
        return;
    }
  ImagenInfo bordes;
  bordes.ancho = imagen->ancho;
  bordes.alto = imagen->alto;
  bordes.canales = 1;
  bordes.pixeles = (unsigned char***)malloc(bordes.alto * sizeof(unsigned char**));
  if (!bordes.pixeles) {
    fprintf(stderr, "Error: Memoria insuficiente para matriz de bordes.\n");
    return;
  }
  for (int y = 0; y < bordes.alto; y++) {
    bordes.pixeles[y] = (unsigned char**)malloc(bordes.ancho * sizeof(unsigned char*));
    if (!bordes.pixeles[y]) {
      fprintf(stderr, "Error: Memoria insuficiente para fila de bordes.\n");
      // Liberar lo que se haya asignado
      for (int k = 0; k < y; k++) {
        for (int x = 0; x < bordes.ancho; x++) free(bordes.pixeles[k][x]);
        free(bordes.pixeles[k]);
      }
      free(bordes.pixeles);
      return;
    }
    for (int x = 0; x < bordes.ancho; x++) {
      bordes.pixeles[y][x] = (unsigned char*)malloc(sizeof(unsigned char));
      if (!bordes.pixeles[y][x]) {
        fprintf(stderr, "Error: Memoria insuficiente para píxel de bordes.\n");
        // Liberar lo que se haya asignado
        for (int k = 0; k <= y; k++) {
          int maxX = (k == y) ? x : bordes.ancho;
          for (int j = 0; j < maxX; j++) free(bordes.pixeles[k][j]);
          free(bordes.pixeles[k]);
        }
        free(bordes.pixeles);
        return;
      }
      bordes.pixeles[y][x][0] = 0;
    }
  }

  int numHilos;
  printf("¿Cuántos hilos deseas usar para Sobel? (2-8): ");
  if (scanf("%d", &numHilos) != 1 || numHilos < 2 || numHilos > 8) {
    printf("Número inválido, usando %d hilos por defecto.\n", GLOBAL_NUM_THREADS);
    numHilos = GLOBAL_NUM_THREADS;
  }
  while (getchar() != '\n'); // Limpiar buffer

  pthread_t hilos[numHilos];
  SobelArgs args[numHilos];
  int filasPorHilo = bordes.alto / numHilos;
  int errorHilo = 0;
  for (int i = 0; i < numHilos; i++) {
    args[i].imagenOriginal = imagen;
    args[i].imagenBordes = &bordes;
    args[i].inicioY = i * filasPorHilo;
    args[i].finalY = (i == numHilos - 1) ? bordes.alto : (i + 1) * filasPorHilo;
    if (args[i].inicioY == 0) args[i].inicioY = 1;
    if (args[i].finalY == bordes.alto) args[i].finalY = bordes.alto - 1;
    if (pthread_create(&hilos[i], NULL, aplicarSobelHilo, &args[i]) != 0) {
      fprintf(stderr, "Error al crear hilo %d\n", i);
      errorHilo = 1;
    }
  }
  for (int i = 0; i < numHilos; i++) {
    if (pthread_join(hilos[i], NULL) != 0) {
      fprintf(stderr, "Error al esperar hilo %d\n", i);
      errorHilo = 1;
    }
  }
  if (errorHilo) {
    printf("Advertencia: Hubo errores en la concurrencia, el resultado puede no ser óptimo.\n");
  }
  printf("Bordes detectados. Puedes guardar la imagen resultante.\n");
  liberarImagen(imagen);
  imagen->ancho = bordes.ancho;
  imagen->alto = bordes.alto;
  imagen->canales = 1;
  imagen->pixeles = bordes.pixeles;
}

// QUÉ: Mostrar el menú interactivo.
// CÓMO: Imprime opciones y espera entrada del usuario.
// POR QUÉ: Proporciona una interfaz simple para interactuar con el programa.
void mostrarMenu() {
  printf("\n--- Plataforma de Edición de Imágenes ---\n");
  printf("1. Cargar imagen PNG\n");
  printf("2. Mostrar matriz de píxeles\n");
  printf("3. Guardar como PNG\n");
  printf("4. Ajustar brillo (+/- valor) concurrentemente\n");
  printf("5. Aplicar desenfoque Gaussiano concurrentemente\n");
  printf("6. Rotar Imagen\n");
  printf("7. Encontrar bordes\n");
  printf("8. Salir\n");
  printf("Opción: ");
}

// QUÉ: Función principal que controla el flujo del programa.
// CÓMO: Maneja entrada CLI, ejecuta el menú en bucle y llama funciones según
// opción. POR QUÉ: Centraliza la lógica y asegura limpieza al salir.
int main(int argc, char *argv[]) {
  ImagenInfo imagen = {0, 0, 0, NULL}; // Inicializar estructura
  char ruta[256] = {0};                // Buffer para ruta de archivo

  // QUÉ: Cargar imagen desde CLI si se pasa.
  // CÓMO: Copia argv[1] y llama cargarImagen.
  // POR QUÉ: Permite ejecución directa con ./img imagen.png.
  if (argc > 1) {
    strncpy(ruta, argv[1], sizeof(ruta) - 1);
    if (!cargarImagen(ruta, &imagen)) {
      return EXIT_FAILURE;
    }
  }

  int opcion;
  while (1) {
    mostrarMenu();
    // QUÉ: Leer opción del usuario.
    // CÓMO: Usa scanf y limpia el buffer para evitar bucles infinitos.
    // POR QUÉ: Manejo robusto de entrada evita errores comunes.
    if (scanf("%d", &opcion) != 1) {
      while (getchar() != '\n')
        ;
      printf("Entrada inválida.\n");
      continue;
    }
    while (getchar() != '\n')
      ; // Limpiar buffer

    switch (opcion) {
    case 1: { // Cargar imagen
      printf("Ingresa la ruta del archivo PNG: ");
      if (fgets(ruta, sizeof(ruta), stdin) == NULL) {
        printf("Error al leer ruta.\n");
        continue;
      }
      ruta[strcspn(ruta, "\n")] = 0; // Eliminar salto de línea
      liberarImagen(&imagen);        // Liberar imagen previa
      if (!cargarImagen(ruta, &imagen)) {
        continue;
      }
      break;
    }
    case 2: // Mostrar matriz
      mostrarMatriz(&imagen);
      break;
    case 3: { // Guardar PNG
      char salida[256];
      printf("Nombre del archivo PNG de salida: ");
      if (fgets(salida, sizeof(salida), stdin) == NULL) {
        printf("Error al leer ruta.\n");
        continue;
      }
      salida[strcspn(salida, "\n")] = 0;
      guardarPNG(&imagen, salida);
      break;
    }
    case 4: { // Ajustar brillo
      int delta;
      printf(
          "Valor de ajuste de brillo (+ para más claro, - para más oscuro): ");
      if (scanf("%d", &delta) != 1) {
        while (getchar() != '\n')
          ;
        printf("Entrada inválida.\n");
        continue;
      }
      while (getchar() != '\n')
        ;
      ajustarBrilloConcurrente(&imagen, delta);
      break;
    }
    case 5: {
      int tamKernel;
      float sigma;
      printf("Tamaño del kernel (debe ser impar, e.g., 3, 5): ");
      if (scanf("%d", &tamKernel) != 1 || tamKernel < 3 || tamKernel % 2 == 0) {
        while (getchar() != '\n')
          ;
        printf("Entrada inválida.\n");
        continue;
      }
      while (getchar() != '\n')
        ;
      printf("Sigma (desviación estándar, e.g., 1.0): ");
      if (scanf("%f", &sigma) != 1 || sigma <= 0) {
        while (getchar() != '\n')
          ;
        printf("Entrada inválida.\n");
        continue;
      }
      while (getchar() != '\n')
        ;
      aplicarDesenfoque(&imagen, 2, tamKernel, sigma);
      break;
    }
    case 6:
      float angulo;

      printf("Ingresa un angulo entre 0 y 360: \n");
      if (scanf("%f", &angulo) != 1 || angulo < 0 || angulo > 360) {
        while (getchar() != '\n')
          ;
        printf("Entrada inválida.\n");
        continue;
      }
      while (getchar() != '\n')
        ;

      rotarImagen(&imagen, angulo);
      break;
      
    case 7: // Encontrar bordes
      detectarBordes(&imagen);
      break;

    case 8: // Salir
      liberarImagen(&imagen);
      printf("¡Adiós!\n");
      return EXIT_SUCCESS;
    default:
      printf("Opción inválida.\n");
    }
  }
  liberarImagen(&imagen);
  return EXIT_SUCCESS;
}

