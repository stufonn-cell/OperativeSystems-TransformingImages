# Procesamiento de Imágenes en C

## Descripción
Este programa permite cargar, visualizar, guardar y procesar imágenes PNG en escala de grises o RGB, usando operaciones como ajuste de brillo, desenfoque gaussiano y rotación. Todo esto con el uso de hilos para reducir distribuir los recursos y el tiempo de ejecución necesario.


## Integrantes del grupo

- Wendy Daniela Benitez Gómez
- Miguel Ángel Cano Salinas
- Delvin José Rodríguez Jimenez
- Jeronimo Acosta Acevedo


## Requisitos
- GCC (compilador de C)
- Las siguientes librerias:
	- `stb_image.h`
	- `stb_image_write.h`

Se debe usar el siguiente comando para descargar las librerias necesarias.
```
wget https://raw.githubusercontent.com/nothings/stb/master/stb_image.h
wget https://raw.githubusercontent.com/nothings/stb/master/stb_image_write.h
```

## Compilación
Ejecuta en la terminal:

```
gcc -o img imagesProcessing.C -pthread -lm
```

- `-pthread`: para soporte de hilos.
- `-lm`: para funciones matemáticas.

## Ejecución

### Modo interactivo
```
./img
```


Sigue las instrucciones del menu para aplicar los métodos.

---
