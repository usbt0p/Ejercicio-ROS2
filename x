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
    docker run --rm -it --name "${CONTAINER_NAME}" \
        --net=host \
        -v "$(pwd)":/auria \
        -w /auria \
        --env DISPLAY="${DISPLAY}" \
        "${IMAGE_NAME}" /bin/bash
fi
