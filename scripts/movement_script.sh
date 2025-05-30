#!/bin/bash

STEPS=40
N_DRONES=20
DRONE_SIZE=1
COLLISIONS=40

mkdir -p data
> data/info.csv
echo "$N_DRONES, $DRONE_SIZE, $COLLISIONS, $STEPS" >> data/info.csv

generate_linear_movement() {
    drone_id=$1
    start_x=$2
    start_y=$3
    start_z=$4
    x_inc=$5
    y_inc=$6
    z_inc=$7
    
    > "data/drone${drone_id}_movement.csv"
    for ((t=0; t<$STEPS; t++)); do
        x=$((start_x + t*x_inc))
        y=$((start_y + t*y_inc))
        z=$((start_z + t*z_inc))
        echo "$x,$y,$z" >> "data/drone${drone_id}_movement.csv"
    done
}

generate_circular_movement() {
    drone_id=$1
    center_x=$2
    center_y=$3
    z=$4
    radius=$5
    direction=$6
    
    > "data/drone${drone_id}_movement.csv"
    for ((t=0; t<$STEPS; t++)); do
        angle=$(echo "scale=6; $direction * $t*2*3.14159/$STEPS" | bc -l)
        x=$(echo "scale=0; $center_x + $radius * c($angle)" | bc -l)
        y=$(echo "scale=0; $center_y + $radius * s($angle)" | bc -l)
        x=${x%.*}
        y=${y%.*}
        echo "$x,$y,$z" >> "data/drone${drone_id}_movement.csv"
    done
}

generate_spiral_movement() {
    drone_id=$1
    center_x=$2
    center_y=$3
    start_z=$4
    radius=$5
    z_inc=$6
    
    > "data/drone${drone_id}_movement.csv"
    for ((t=0; t<$STEPS; t++)); do
        angle=$(echo "scale=6; $t*2*3.14159/$STEPS" | bc -l)
        r=$(echo "scale=1; $radius * (1 + $t/($STEPS*2))" | bc -l)
        x=$(echo "scale=0; $center_x + $r * c($angle)" | bc -l)
        y=$(echo "scale=0; $center_y + $r * s($angle)" | bc -l)
        z=$((start_z + t*z_inc))
        x=${x%.*}
        y=${y%.*}
        echo "$x,$y,$z" >> "data/drone${drone_id}_movement.csv"
    done
}

generate_oscillating_movement() {
    drone_id=$1
    center_x=$2
    center_y=$3
    center_z=$4
    amplitude_x=$5
    amplitude_y=$6
    amplitude_z=$7
    
    > "data/drone${drone_id}_movement.csv"
    for ((t=0; t<$STEPS; t++)); do
        angle=$(echo "scale=6; $t*2*3.14159/$STEPS" | bc -l)
        x=$(echo "scale=0; $center_x + $amplitude_x * c($angle)" | bc -l)
        y=$(echo "scale=0; $center_y + $amplitude_y * s($angle*2)" | bc -l)
        z=$(echo "scale=0; $center_z + $amplitude_z * s($angle/2)" | bc -l)
        x=${x%.*}
        y=${y%.*}
        z=${z%.*}
        echo "$x,$y,$z" >> "data/drone${drone_id}_movement.csv"
    done
}

for ((i=1; i<=6; i++)); do
    start_x=$((RANDOM % 50))
    start_y=$((RANDOM % 50))
    start_z=$((RANDOM % 10))
    x_inc=$((RANDOM % 5 - 2))
    y_inc=$((RANDOM % 5 - 2))
    z_inc=$((RANDOM % 3 - 1))
    if [ $x_inc -eq 0 ] && [ $y_inc -eq 0 ]; then
        x_inc=1
    fi
    generate_linear_movement $i $start_x $start_y $start_z $x_inc $y_inc $z_inc
done

for ((i=7; i<=12; i++)); do
    if [ $i -lt 9 ]; then
        center_x=25
        center_y=25
    else
        center_x=$((RANDOM % 60 + 10))
        center_y=$((RANDOM % 60 + 10))
    fi
    z=$((RANDOM % 15 + 1))
    radius=$((RANDOM % 15 + 5))
    direction=$((RANDOM % 2 * 2 - 1))
    generate_circular_movement $i $center_x $center_y $z $radius $direction
done

# Spiral movement (drones 13-16)
for ((i=13; i<=16; i++)); do
    if [ $i -lt 15 ]; then
        center_x=50
        center_y=50
    else
        center_x=$((RANDOM % 70 + 15))
        center_y=$((RANDOM % 70 + 15))
    fi
    start_z=$((RANDOM % 10))
    radius=$((RANDOM % 10 + 5))
    z_inc=$((RANDOM % 2 + 1))
    generate_spiral_movement $i $center_x $center_y $start_z $radius $z_inc
done

for ((i=17; i<=20; i++)); do
    if [ $i -lt 19 ]; then
        center_x=35
        center_y=35
    else
        center_x=$((RANDOM % 50 + 25))
        center_y=$((RANDOM % 50 + 25))
    fi
    center_z=$((RANDOM % 15 + 5))
    amplitude_x=$((RANDOM % 15 + 5))
    amplitude_y=$((RANDOM % 15 + 5))
    amplitude_z=$((RANDOM % 5 + 1))
    generate_oscillating_movement $i $center_x $center_y $center_z $amplitude_x $amplitude_y $amplitude_z
done

echo "Generated data for $N_DRONES drones successfully!"
