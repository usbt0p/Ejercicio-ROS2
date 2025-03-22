#!/bin/bash

if ! docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^$IMAGE_NAME$"; then
    read -p "La imagen $IMAGE_NAME no existe. ¿Quieres construirla? [y/n]: " response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        echo "Construyendo la imagen..."
    	docker build -f Dockerfile.dev -t cursos_img .
    else
        echo "Operación cancelada."
        exit 1
    fi
fi

# Verificar si el contenedor ya está en ejecución
if docker ps --format '{{.Names}}' | grep -q "^$CONTAINER_NAME$"; then
    echo "Adjuntando al contenedor en ejecución..."
    docker exec -it $CONTAINER_NAME /bin/bash
else
    echo "Iniciando un nuevo contenedor..."
    docker run --rm -it --name cursos_container \
        --net=host \
        --env DISPLAY=$DISPLAY \
        cursos_img /bin/bash
fi

