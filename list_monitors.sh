#!/bin/bash

echo "========================================="
echo "        Monitores Conectados (XRANDR)    "
echo "========================================="

# Pega apenas as linhas de monitores conectados
xrandr | grep " connected" | while read -r line; do
    # Extrai o nome (primeira palavra)
    name=$(echo "$line" | awk '{print $1}')
    
    # Verifica se é o primário
    primary=$(echo "$line" | grep -o "primary")
    
    # Extrai o bloco de resolução+posição, ex: 1920x1080+0+0
    res_pos=$(echo "$line" | grep -oE "[0-9]+x[0-9]+\+[0-9]+\+[0-9]+")
    
    echo "Nome (ID): $name"
    
    if [ ! -z "$primary" ]; then
        echo "Status:    Primário"
    else
        echo "Status:    Secundário"
    fi
    
    if [ ! -z "$res_pos" ]; then
        # Separa a resolução e as coordenadas
        res=$(echo "$res_pos" | cut -d'+' -f1)
        pos_x=$(echo "$res_pos" | cut -d'+' -f2)
        pos_y=$(echo "$res_pos" | cut -d'+' -f3)
        echo "Resolução: $res"
        echo "Posição X: $pos_x"
        echo "Posição Y: $pos_y"
    else
        echo "Resolução: Desligado / Sem sinal"
    fi
    echo "-----------------------------------------"
done
