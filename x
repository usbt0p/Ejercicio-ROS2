#!/bin/bash

# Define los nombres de imagen y contenedor
IMAGE_NAME="cursos_img"
CONTAINER_NAME="cursos_container"

# Verificar si la imagen existe
if ! docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}:latest$"; then
    read -p "La imagen ${IMAGE_NAME} no existe. ¿Quieres construirla? [y/n]: " response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        echo "Construyendo la imagen..."
        docker build -f Dockerfile.dev -t "${IMAGE_NAME}" .
    else
        echo "Operación cancelada."
        exit 1
    fi
fi

# Verificar si el contenedor ya está en ejecución
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Adjuntando al contenedor en ejecución..."
    docker exec -it "${CONTAINER_NAME}" /bin/bash
else
    echo "Iniciando un nuevo contenedor..."
    docker run --rm -it --name "${CONTAINER_NAME}" \
        --net=host \
        --env DISPLAY="${DISPLAY}" \
        "${IMAGE_NAME}" /bin/bash
fi
