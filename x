#!/bin/bash

IMAGE_NAME="cursos_img"
CONTAINER_NAME="cursos_container"
DOCKERFILE="Dockerfile.dev"
HASH_FILE=".dockerfile.hash"
CURRENT_HASH=$(sha256sum "$DOCKERFILE" | cut -d ' ' -f1)
PREVIOUS_HASH=$(cat "$HASH_FILE" 2>/dev/null)


if [[ "$CURRENT_HASH" != "$PREVIOUS_HASH" ]] || [[ -z $(docker images -q "$IMAGE_NAME") ]]; then
    read -p "Dockerfile modificado o la imagen no existe. ¿Quieres construirlo? [Y/n] " response
    if [[ -z "$response" || "$response" =~ ^[Yy]$ ]]; then
        docker build -f "$DOCKERFILE" -t "$IMAGE_NAME" .
        echo "$CURRENT_HASH" > "$HASH_FILE"
    else
        echo "Operación cancelada."
        exit 1
    fi
fi

if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Adjuntando al contenedor en ejecución..."
    docker exec -it "${CONTAINER_NAME}" /bin/bash
else
    echo "Iniciando un nuevo contenedor..."

    xhost +local:root > /dev/null
    docker run --rm -it --name "${CONTAINER_NAME}" \
        --net=host \
        -v "$(pwd)":/home/auria \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --env DISPLAY="${DISPLAY}" \
        --env QT_X11_NO_MITSHM=1 \
        --env LIBGL_ALWAYS_SOFTWARE=1 \
        --device /dev/dri \
        "${IMAGE_NAME}" /bin/bash
fi
